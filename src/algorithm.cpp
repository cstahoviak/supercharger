/**
 * @file algorithm.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "algorithm.h"
#include "logging.h"
#include "math.h"
// Need to include planner.h here to avoid "pointer or reference to incomplete
// type is not allowed" errors erlated to using the route_planner_ pointer.
#include "planner.h"

#include <algorithm>
#include <map>

namespace supercharger
{
  // NOTE: Do NOT repeat the 'static keyword at the cpp file level
  // NOTE: I think this must be defined at the cpp file level because otherwise
  // I get a "not declared in this scope error" related to the derived planning
  // algorithm types (BruteForce, Dijkstra's, etc.). 
  std::unique_ptr<PlanningAlgorithm> PlanningAlgorithm::GetPlanner(
    AlgorithmType&& algo_type,
    CostFunctionType&& cost_type = CostFunctionType::NONE)
  {
    switch ( algo_type )
    {
      case AlgorithmType::BRUTE_FORCE:
        // NOTE: There is an important difference between public and protected
        // inheritance when it comes to the compiler "being aware" that a
        // unique_ptr of the base class (the return type of this function) can
        // be initialized from unique_ptr of the derived class: public
        // inheritance allows this, but protected inheritance does not. But why?
        return std::make_unique<BruteForce>(cost_type);

      case AlgorithmType::DIJKSTRAS:
        // return std::make_unique<Dijstras>();
        return std::unique_ptr<PlanningAlgorithm>(nullptr);

      case AlgorithmType::ASTAR:
        return std::unique_ptr<PlanningAlgorithm>(nullptr);
        // return std::make_unique<AStar>();
      
      default:
        return std::unique_ptr<PlanningAlgorithm>(nullptr);
    }
  }

  double BruteForce::ComputeChargeTime(
    const Stop& current_stop, const Charger* const next_charger) const
  {
    LOG("Charge rate at '"<< current_stop.charger->name << "': " << 
      current_stop.charger->rate);
    // Compute the distance to the next charger
    double current_to_next = great_circle_distance(
      current_stop.charger->lat,
      current_stop.charger->lon,
      next_charger->lat,
      next_charger->lon
    );

    // Compute the charge time required to make it to the next charger.
    // NOTE that we're charging the car only enough to make it to the next stop.
    return (current_to_next - current_stop.range) / current_stop.charger->rate;
  }

  double BruteForce::ComputeCost(const Charger* const current, const Charger* const candidate) const {
    // Define the cost
    double cost{0};
    
    switch ( type_ )
    {
      case CostFcnType::MINIMIZE_DIST_REMAINING:
      {
        // The "cost" is the distance from the candidate charger to the
        // destination charger.
        cost = great_circle_distance(
          candidate->lat,
          candidate->lon,
          route_planner_->destination()->lat,
          route_planner_->destination()->lon
        );
        break;
      }
      
      case CostFcnType::MINIMIZE_TIME_REMAINING:
      {
        // Compute distances
        double candidate_to_destination = great_circle_distance(
          candidate->lat,
          candidate->lon,
          route_planner_->destination()->lat,
          route_planner_->destination()->lon
        );
        double current_to_candidate = great_circle_distance(
          current->lat, current->lon, candidate->lat, candidate->lon
        );

        // Compute times
        double time_to_destination = 
          candidate_to_destination / route_planner_->speed();
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
  void BruteForce::PlanRoute(std::vector<Stop>& route) {
    // Get the current stop
    Stop& current_stop = route.back();
    LOG("\nCurrent route: " << route);

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

    for ( const auto& [name, charger] : route_planner_->network() ) {
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
        route_planner_->destination()->lat,
        route_planner_->destination()->lon
      );
      candidate_to_dest = great_circle_distance(
        charger->lat,
        charger->lon,
        route_planner_->destination()->lat,
        route_planner_->destination()->lon
      );

      // If candidate charger is within the maximum range of the vehicle, add
      // the candidate charger to "reachable" set
      if ( current_to_candidate <= route_planner_->max_range() ) {
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
        double cost = ComputeCost(current_stop.charger, charger);

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
    if ( std::find(candidate_names.begin(), candidate_names.end(), route_planner_->destination()->name) != candidate_names.end() ) {
      // Compute the remaining range after arriving at the destination
      double range_remaining = great_circle_distance(
        current_stop.charger->lat,
        current_stop.charger->lon,
        route_planner_->destination()->lat,
        route_planner_->destination()->lon
      );

      // Compute the charge time to make it to the final destination
      current_stop.duration = 
        ComputeChargeTime(current_stop, route_planner_->destination());
      LOG("Charge time at '" << current_stop.charger->name << "':" <<
        current_stop.duration);

      // Update the total trip cost
      total_cost_ += current_stop.duration +
        great_circle_distance(
          current_stop.charger->lat,
          current_stop.charger->lon,
          route_planner_->destination()->lat,
          route_planner_->destination()->lon
          ) / route_planner_->speed();

      // Add the destination as the final stop on the route
      route.emplace_back(
        route_planner_->network().at(route_planner_->destination()->name),
        0,
        range_remaining,
        &current_stop);

      return;
    }

    // Choose the next charger such that it minimizes the cost
    double key = candidates.begin()->first;
    Charger* next_charger = candidates.at(key);

    double current_to_next = great_circle_distance(
      current_stop.charger->lat,
      current_stop.charger->lon,
      next_charger->lat,
      next_charger->lon
    );
    total_cost_ += current_to_next / route_planner_->speed();

    // If we've arrived at the current stop with less than the maximum range
    // (which should be true for every stop that's not the origin), determine
    // charge time to make it to the next stop
    LOG("Current Range at '" << current_stop.charger->name << "': " << 
      current_stop.range);
    LOG("Distance to '" << next_charger->name << "': " << current_to_next);
    
    if ( current_stop.range < route_planner_->max_range() ) {
      current_stop.duration = 
        ComputeChargeTime(current_stop, next_charger);
      LOG("Charge time at '" << current_stop.charger->name << "':" <<
        current_stop.duration);

      total_cost_ += current_stop.duration;
    }

    // Add the next stop to the route and continue iteration. Note that the
    // charge duration at the next stop will be computed on the next iteration.
    double range_remaining = current_stop.range + 
      (current_stop.duration * current_stop.charger->rate) -
      great_circle_distance(
        current_stop.charger->lat,
        current_stop.charger->lon,
        next_charger->lat,
        next_charger->lon
      );
    route.emplace_back(
      next_charger, static_cast<double>(NULL), range_remaining);
    PlanRoute(route);

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

  void Dijkstras::PlanRoute(std::vector<Stop>& route) {
    // 1. Mark all "nodes"/stops as unvisited

    // 2. Assign a "distance to start" value to each node in the network.
    // Initially, this value will be "infinity" since no path is yet known to
    // these "unvisited" nodes.

    // 3. From the unvisited set, select the current node to be the one with
    // the smallest known distance from the origin. If the current node is the
    // destination node, we're done; otherwise, continue to find the shortest
    // path to all reachable nodes.

    // 4. For the current node, consider all of its unvisited neighbors and
    // update their distance through the current node. Compare the newly
    // calculated distance to the one currently assigned to the neighbor, and
    // assign the neighbor the smaller distance.

    // 5. Once we've considered all the unvisited neighbors of the current node,
    // mark the current node as visited and remove it from the unvisited set.

    // 6. Return to step 3.

    // 7. Once the loop (steps 3-5) exits, every node will contain the shortest
    // distance from the start node.
    return;
  }

  void AStar::PlanRoute(std::vector<Stop>& route) {
    return;
  }

} // end namespace supercharger