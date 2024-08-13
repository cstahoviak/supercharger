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
#include "math.h"
#include "planner.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

namespace supercharger
{
  std::ostream& operator<<(std::ostream& stream, const std::vector<Stop>& route) {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const Stop& stop : route ) {
      stream << stop.charger->name;
      if ( stop.duration > 0 ) {
        stream << ", " << std::setprecision(5) << stop.duration;
      }
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  RoutePlanner::RoutePlanner() {
    // Create the network map
    for ( Charger& charger : supercharger::network ) {
      // const std::pair<std::unordered_map<std::string, Charger>::iterator, bool> pair =
      const auto& pair = network_.try_emplace(charger.name, &charger);
      if ( !pair.second ) {
        LOG("Charger '" << charger.name << "' already exists in the network. "
          << "Skipping.");
      }
    }
  }

  std::vector<Stop> RoutePlanner::PlanRoute(const std::string& origin, const std::string& destination, CostType cost_type) {
    // Initialize the route
    std::vector<Stop> route = InitializeRoute_(origin, destination);

    LOG("Planning route between '" << origin_->name << "' and '" <<
      destination_->name << "'");

    // Plan the route
    BruteForce_(route, cost_type);

    if ( route.back().charger->name == destination_->name ) {
      LOG("\nSolution found!");
    }
    else {
      LOG("\nSearch terminated. Solution not found.");
    }

    // TODO: Backtrace to determine the route (doesn't apply to current
    // solution method).
    
    return route;
  }

  std::vector<Stop> RoutePlanner::InitializeRoute_(const std::string& origin, const std::string& destination) {
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

    // Initialize the route and add the origin to the route
    std::vector<Stop> route;
    route.emplace_back(origin_, 0, max_range_, nullptr);
    return route;
  }

  double RoutePlanner::ComputeChargeTime_(const Charger* const current, const Charger* const next) const {
    // Compute the distance to the next charger
    double current_to_next = great_circle_distance(
      current->lat, current->lon, next->lat, next->lon);

    // Compute the charge time required to make it to the next charger.
    // NOTE that we're charging the car only enough to make it to the next stop.
    return current_to_next / current->rate;
  }

  double RoutePlanner::ComputeCost_(const Charger* const current, const Charger* const candidate, CostType type) const {
    // Define the cost
    double cost{0};
    
    switch ( type )
    {
      case CostType::MINIMIZE_DIST_REMAINING:
      {
        // The "cost" is the distance from the candidate charger to the
        // destination charger.
        cost = great_circle_distance(
          candidate->lat, candidate->lon, destination_->lat, destination_->lon
        );
        break;
      }
      
      case CostType::MINIMIZE_TIME_REMAINING:
      {
        // Compute distances
        double candidate_to_destination = great_circle_distance(
          candidate->lat, candidate->lon, destination_->lat, destination_->lon
        );
        double current_to_candidate = great_circle_distance(
          current->lat, current->lon, candidate->lat, candidate->lon
        );

        // Compute times
        double time_to_destination = candidate_to_destination / speed_;
        double time_to_charge = current_to_candidate / candidate->rate;

        // The cost is the total time to drive the remaining distance between
        // the candidate charger and the destination + the time to fully charge
        // at the candidate charger.
        cost = weight_time_to_destination_ * time_to_destination + 
          weight_time_to_charge_ * time_to_charge;
        break;
      }
      
      default:
        break;
    }
    return cost;
  }

  /**
   * @brief The "brute force" route planner.
   * 
   * TODO: Consider using std::unordered_set to store the "reachable" (chargers 
   * that are within the current range of the vehicle post-charging) and 
   * "closer to" (chargers that are closer to the destination station than the 
   * current charger) sets of candidate chargers. Then, the actual set of
   * candidate chargers will be the intersection (std::set_intersection) of the
   * "reachable" and "closer to" sets. I beleive this will require defining
   * both operator== and a hash function for the supercharger::Charger class.
   * 
   * @param current_stop 
   * @return Stop 
   */
  void RoutePlanner::BruteForce_(std::vector<Stop>& route, CostType cost_type) const {
    // Get the current stop
    Stop& current_stop = route.back();
    LOG("Current route: " << route);

    // Use a map to store the candidate chargers, i.e. the next possible
    // chargers on the route, sorted by distance to the destination
    std::map<double, Charger*> candidates;
    std::vector<std::string> candidate_names;
    bool is_reachable{false};
    bool is_closer{false};

    // Store relative distance measurements
    double current_to_candidate{0};
    double current_to_dest{0};
    double candidate_to_dest{0};

    for ( auto& [name, charger] : network_ ) {
      is_reachable = false;
      is_closer = false;

      // Compute distances
      current_to_candidate = great_circle_distance(
        current_stop.charger->lat,
        current_stop.charger->lon,
        charger->lat,
        charger->lon
      );
      current_to_dest = great_circle_distance(
        current_stop.charger->lat,
        current_stop.charger->lon,
        destination_->lat,
        destination_->lon
      );
      candidate_to_dest = great_circle_distance(
        charger->lat,
        charger->lon,
        destination_->lat,
        destination_->lon
      );

      // If candidate charger is within the maximum range of the vehicle, add
      // the candidate charger to "reachable" set
      if ( current_to_candidate <= max_range_ ) {
        is_reachable = true;
      }

      // If the candidate charger is closer to the destination than the current
      // charger, add the candidate charger to the "closer to" set
      if ( candidate_to_dest < current_to_dest ) {
        is_closer = true;
      }

      // If the charger is reachable, and it's closer to the destination than
      // the current charger, add it to map of candidate chargers
      if ( is_reachable && is_closer ) {
        // Compute the cost for the candidate charger
        double cost = ComputeCost_(current_stop.charger, charger, cost_type);

        // Add the charger to the map of candidate chargers
        const auto& pair2 = candidates.try_emplace(cost, charger);
        candidate_names.push_back(charger->name);
      }
    }

    // TODO: Compute the intersection of the "reachable" and "closer to" sets
    // std::unordered_set<Charger> candidate_chargers;
    // std::set_intersection(reachable.begin(), reachable.end(),
    //   closer_to.begin(), closer_to.end(),
    //              std::inserter(candidate_chargers, candidate_chargers.begin()));

    // If the destination is in the set of candidate chargers, we can go to the
    // destination on our remaining charge and we're done!
    if ( std::find(candidate_names.begin(), candidate_names.end(), destination_->name) != candidate_names.end() ) {
      // Compute the remaining range after arriving at the destination
      double range_remaining = great_circle_distance(
        current_stop.charger->lat,
        current_stop.charger->lon,
        destination_->lat,
        destination_->lon
      );

      // Compute the charge time to make it to the final destination
      current_stop.duration = 
        ComputeChargeTime_(current_stop.charger, destination_);

      // Add the destination as the final stop on the route
      route.emplace_back(
        network_.at(destination_->name), 0, range_remaining, &current_stop);

      return;
    }

    // Choose the next charger such that it minimizes the cost
    double key = candidates.begin()->first;
    Charger* next_charger = candidates.at(key);

    // If we've arrived at the current stop with less than the maximum range
    // (which should be true for every stop that's not the origin), determine
    // charge time to make it to the next stop
    if ( current_stop.range < max_range_ ) {
      current_stop.duration = 
        ComputeChargeTime_(current_stop.charger, next_charger);
    }

    // Add the next stop to the route and continue iteration. Note that the
    // charge duration at the next stop will be computed on the next iteration.
    route.emplace_back(
      next_charger, static_cast<double>(NULL), 0, &current_stop);
    BruteForce_(route, cost_type);

    // Continue iteration with next stop
    // Stop next{next_charger, static_cast<double>(NULL), 0, &current_stop};
    // route.push_back(BruteForce_(next, cost_type));
    // route.push_back(std::move(BruteForce_(next, cost_type)));

    // NOTE: My initial thought was to not choose a single "next charger", but
    // recursively call BruteForce_ on ALL candidate chargers. Thus allowing the
    // search space to grow beyond a single charger at each iteration. I have no
    // idea if this is even feasible, or if it can be accomplished via
    // recursion. Below is the code I wrote to attempt that (it didn't work).
    
    // If the destination is not in the reachable set, for each Charger in the
    // reachable set, compute its reachable set. Repeat until the destination
    // charger is in the reachable set.
    // for ( const auto& [name, charger] : candidate_chargers ) {
    //   // Compute the charge time to return to max charge (Assume for now that 
    //   // the vehicle leaves the current stop with a full charge)
    //   double dist = ComputeDistance(*(current_stop.charger), *charger);
    //   double charge_duration = (max_range_ - dist) / charger->rate;

    //   // Recursively call the brute force algorithm until the destination is
    //   // reached
    //   Stop next_stop = 
    //     { network_.at(charger->name), charge_duration, max_range_, &current_stop };
    //   BruteForce_(next_stop);
    // }

    return;
  }

  /**
   * @brief Implements Dijkstra's algorithm.
   */
  void RoutePlanner::Dijkstra_(std::vector<Stop>& route) const {
    return;
  }

  /**
   * @brief Implements the A* search algorithm. A* makes sense in this scenario
   * because the total cost of the route is computed as a function of both the
   * distance (total time traveled) from the current node to the start node as 
   * well as a heuristic that, in our case, should be a function of the charge
   * rate.
   */
  void RoutePlanner::AStar_(std::vector<Stop>& route) const {
    return;
  }
} // end namespace supercharger