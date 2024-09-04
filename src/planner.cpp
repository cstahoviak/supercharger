/**
 * @file planner.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/algorithm.h"
#include "logging.h"
#include "math.h"
#include "planner.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>

namespace supercharger
{
  /**
   * @brief Formats the final route output to comply with the format expected
   * by the provided "checker" executables.
   * 
   * @param stream 
   * @param route 
   * @return std::ostream& 
   */
  std::ostream& operator<<(std::ostream& stream, const std::vector<Node>& route) {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const Node& node : route ) {
      stream << node;
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  std::ostream& operator<<(std::ostream& stream, const std::vector<Node*>& route) {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const Node* const node : route ) {
      stream << *node;
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  RoutePlanner::RoutePlanner(AlgoType&& algo_type, CostFcnType&& cost_type) {
    // Create the network map (must do this before creating the planning algo).
    for ( Charger& charger : supercharger::network ) {
      // const std::pair<std::unordered_map<std::string, Charger>::iterator, bool> pair =
      const auto& pair = network_.try_emplace(charger.name, &charger);
      if ( !pair.second ) {
        DEBUG("Charger '" << charger.name << "' already exists in the network. "
          << "Skipping.");
      }
    }

    // Create the planning algorithm
    // NOTE: Even though this function accepts 'algo_type' and 'cost_type as 
    // r-value references, once we're within the scope of this function, they
    // become l-values (pretty odd, right?). So we need to use std::move() to 
    // cast the l-values 'algo_type' and 'cost_type' as r-values to "move" them
    // to PlanningAlgorithm::GetPlanner.
    planning_algo_ = PlanningAlgorithm::GetPlanningAlgorithm(
      this, std::move(algo_type), std::move(cost_type)
    );
  }

  PlannerResult RoutePlanner::PlanRoute(
    const std::string& origin, const std::string& destination)
  {
    // Initialize the route
    Initialize_(origin, destination);

    // Plan the route
    DEBUG("Planning route between '" << origin << "' and '" << destination << 
      "'.");
    PlannerResult result = planning_algo_.get()->PlanRoute(origin, destination);

    if ( result.route.back().charger->name == destination_->name ) {
      DEBUG("Solution found!");
    }
    else {
      DEBUG("Search terminated. Solution not found.");
    }
    
    return result;
  }

  PlannerResult RoutePlanner::OptimizeRoute(const std::vector<Node>& route) const {
    // For debug purposes, print the charger rate at each node
    for ( const Node& node : route ) {
      DEBUG("'" << node.charger->name << "' charger rate: " <<
        node.charger->rate);
    }
    
    // Find the node with the fastest charger in the route (that's not the
    // first, last or second to last node in the route).
    double max_rate{0};
    size_t argmax{0};
    for ( size_t idx = 1; idx < route.size() - 2; idx++) {
      if ( route.at(idx).charger->rate > max_rate ) {
        max_rate = route.at(idx).charger->rate;
        argmax = idx;
      }
    }
    
    const Node& max_node = route.at(argmax);

    DEBUG("Charger at '" << max_node.charger->name << "' with a rate of " <<
      max_node.charger->rate << " km/hr is the fastest charger on the route.");

    // The charge time at the "max-rate" node will be a function of the
    // distance remaining to the destination.
    double dist_remaining{0};
    for ( size_t idx = argmax + 1; idx < route.size(); idx++ ) {
      const Node& prev = route.at(idx - 1);
      const Node& current = route.at(idx);
      dist_remaining += compute_distance(prev.charger, current.charger);
    }
    DEBUG("Distance remaining between '" << max_node.charger->name <<
      "' and '" << destination_->name << "': " << dist_remaining << "km.");

    // If the distance remaining is less than the max range of the vehicle, only
    // charge the vehicle enough to make it the destination. Otherwise, the
    // distance remaining is greater than the maximum range of the vihicle, so
    // we can charge the vehicle fully. 
    double range_on_departure = ( dist_remaining <= max_range_ ) ? 
      dist_remaining - max_node.departure_range : max_range_;

    // Compute the charging duration at the max-rate node.
    double duration = range_on_departure / max_node.charger->rate;

    // For the max-rate node, update the newly calculated charge duration.
    DEBUG("Charging for " << duration << " at '" <<
      max_node.charger->name << "'.");
    DEBUG("Leaving '" << max_node.charger->name << "' with a range of " <<
      range_on_departure << "km.");

    // Initialize the optimized route as a copy of the provided route and
    // update the charge duration at the newly-optimized max-rate node.
    std::vector<Node> optimized = route;
    Node& optimized_max_node = optimized.at(argmax);
    optimized_max_node.duration = duration;

    // Update the route cost at the newly-optimized max-rate node
    optimized_max_node.cost = optimized.at(argmax - 1).cost + duration + 
      compute_distance(optimized.at(argmax - 1), max_node) / speed_;

    // Update the optimized route from the max-rate node to the end.
    double range_remaining = range_on_departure;
    for ( size_t idx = argmax + 1; idx < optimized.size(); idx++ ) {
      const Node& prev = optimized.at(idx - 1);
      Node& current = optimized.at(idx);

      // For all remaining nodes on the route, update the range remaining
      // after arriving at the node.
      range_remaining -= compute_distance(prev, current);
      current.arrival_range = range_remaining;

      // Update the total cost of the optimized route.
      current.cost = prev.cost + compute_distance(prev, current) / speed_;

      if ( idx < optimized.size() - 1 ) {
        // For all nodes not the destination, update the charge duration if
        // the range remaining is less than the distance to the next node.
        Node& next = optimized.at(idx + 1);
        double dist_to_next = compute_distance(current, next);
        if ( dist_to_next > range_remaining ) {
          // Charge just long enough to make it to the next node.
          current.duration =
            (dist_to_next - current.departure_range) / current.charger->rate;
          DEBUG("Updating charge duration at '" << current.charger->name <<
            "' from " << route.at(idx).duration << " hrs to " <<
            current.duration << " hrs.");
          current.cost += current.duration;
        }
      }
    }
    return { optimized, optimized.back().cost };
  }

  void RoutePlanner::Initialize_(
    const std::string& origin, const std::string& destination)
  {
    // Store the origin and destination chargers
    try {
      origin_ = network_.at(origin); 
    }
    catch( const std::out_of_range& e) {
      std::ostringstream os;
      os << "Initial charger '" << origin << "' not in network.";
      throw std::out_of_range(os.str());
    };

    try {
      destination_ = network_.at(destination); 
    }
    catch( const std::out_of_range& e) {
      std::ostringstream os;
      os << "Destination charger '" << destination << "' not in network.";
      throw std::out_of_range(os.str());
    };

    // TODO: max_range_ should actually be greater than or equal to the
    // starting node's nearest neighbor.
    if ( max_range_ <= 0 ) {
      std::ostringstream os;
      os << "The vehicle's maximum range value of " << max_range_ << "km " <<
        "is invalid. The vehicle's max range must be greater than or equal " <<
        "to the nearest neighbor of the starting node.";
      throw std::runtime_error(os.str());
    }

    if ( speed_ <= 0 ) {
      std::ostringstream os;
      os << "The vehicle's speed value of " << speed_ << " km/hr is " <<
      "invalid. The vehicle's speed must be greater than zero.";
      throw std::runtime_error(os.str());
    }
  }
} // end namespace supercharger