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
#include <iostream>
#include <map>
#include <sstream>
#include <unordered_set>

namespace supercharger
{
  std::ostream& operator<<(std::ostream& stream, const std::vector<std::optional<Stop>>& route) {
    size_t sz = route.size();
    size_t idx = 0;
    for ( auto& stop : route ) {
      if ( stop.has_value() ) {
        stream << stop.value().charger->name;
        if ( stop.value().duration > 0 ) {
          stream << ", " << stop.value().duration;
        }
        if ( idx < sz - 1 ) {
          stream << ", ";
        }
      }
      idx++;
    }
    return stream;
  }

  RoutePlanner::RoutePlanner(std::string& origin, std::string& destination) {
    // Create the network map
    for ( Charger& charger : supercharger::network ) {
      // const std::pair<std::unordered_map<std::string, Charger>::iterator, bool> pair =
      const auto& pair = network_.try_emplace(charger.name, &charger);
      if ( !pair.second ) {
        LOG("Charger '" << charger.name << "' already exists in the network. "
          << "Skipping.");
      }
    }

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
  }

  std::vector<std::optional<Stop>> RoutePlanner::PlanRoute(CostType cost_type) {
    LOG("Planning route between '" << origin_->name << "' and '" <<
      destination_->name << "'");

    // Plan the route
    Stop origin{origin_, 0, max_range_, nullptr};
    std::optional<Stop> final_stop = BruteForce_(origin, cost_type);

    // Clean up the route by removing empty Stops (caused by brute force 
    // recursion method)
    while ( !route_.back().has_value() ) {
      route_.pop_back();
    }

    if ( route_.back().value().charger->name == destination_->name ) {
      LOG("\nSolution found!");
    }
    else {
      LOG("\nSearch terminated. Solution not found.");
    }

    // TODO: Backtrace to determine the route (doesn't apply to current
    // solution method)
    
    return route_;
  }

  double RoutePlanner::BruteForceCost_(const Charger* const current, const Charger* const candidate, CostType type) const {
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
        cost = time_to_destination + time_to_charge;
        break;
      }
      
      default:
      {
        break;
      }
    }
    return cost;
  }

  /**
   * @brief The "brute force route planner."
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
  std::optional<Stop> RoutePlanner::BruteForce_(Stop& current_stop, CostType cost_type) {
    // Add the current stop to the route
    route_.push_back(current_stop);
    LOG("Current route: " << route_);

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

      // If candidate charger is within the current range of the vehicle, add
      // the candidate charger to "reachable" set
      if ( current_to_candidate <= current_stop.range ) {
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
        double cost = BruteForceCost_(current_stop.charger, charger, cost_type);

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

      // Return the final Stop
      Stop final = 
        {network_.at(destination_->name), 0, range_remaining, &current_stop};
      return final;
    }

    // Choose the next charger such that it minimizes the cost
    double key = candidates.begin()->first;
    Charger* next_charger = candidates.at(key);

    // Compute the charge time to return to max charge (Assume for now that 
    // the vehicle leaves the current stop with a full charge)
    double dist = great_circle_distance(
      current_stop.charger->lat,
      current_stop.charger->lon,
      next_charger->lat,
      next_charger->lon
    );
    double charge_duration = dist / next_charger->rate;

    Stop* parent = &current_stop;

    // Continue iteration with next stop
    Stop next{next_charger, charge_duration, max_range_, parent};
    route_.emplace_back(BruteForce_(next, cost_type));

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

    // If no solution is found, return empty
    return {};
  }

  /**
   * @brief Implements Dijkstra's algorithm.
   */
  void RoutePlanner::Dijkstra_() {
    return;
  }

  /**
   * @brief Implements the A* search algorithm. A* makes sense in this scenario
   * because the total cost of the route is computed as a function of both the
   * distance (total time traveled) from the current node to the start node as 
   * well as a heuristic that, in our case, should be a function of the charge
   * rate.
   */
  void RoutePlanner::AStar_() {
    return;
  }
} // end namespace supercharger