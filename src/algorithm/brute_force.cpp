/**
 * @file brute_force.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/brute_force.h"
#include "logging.h"
// Need to include planner.h here to avoid "pointer or reference to incomplete
// type is not allowed" errors erlated to using the route_planner_ pointer.
#include "planner.h"

#include <algorithm>
#include <map>
#include <stdexcept>


namespace supercharger
{
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
   * @param route 
   */
  void BruteForce::PlanRoute(std::vector<Stop>& route) {
    // Get the current stop
    Stop& current_stop = route.back();
    LOG("Current route: " << route);

    // Create a local alias for the destination charger
    const Charger* const destination = route_planner_->destination();

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
      current_to_candidate = ComputeDistance_(current_stop.charger, charger);
      current_to_dest = ComputeDistance_(current_stop.charger, destination);
      candidate_to_dest = ComputeDistance_(charger, destination);

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
        double cost = ComputeCost_(current_stop.charger, charger);

        // Add the charger to the map of candidate chargers
        const auto& pair = candidates.try_emplace(cost, charger);
        candidate_names.push_back(charger->name);
      }
    }

    // TODO: Compute the intersection of the "reachable" and "closer to" sets
    // std::unordered_set<Charger> candidate_chargers;
    // std::set_intersection(reachable.begin(), reachable.end(),
    //   closer_to.begin(), closer_to.end(),
    //   std::inserter(candidate_chargers, candidate_chargers.begin()));

    // If the destination is in the set of candidate chargers, we can go to the
    // destination on our remaining charge and we're done!
    if ( std::find(candidate_names.begin(), candidate_names.end(), destination->name) != candidate_names.end() ) {
      // Compute the charge time to make it to the final destination
      current_stop.duration = ComputeChargeTime_(current_stop, destination);

      // Update the total cost at the current stop
      UpdateRouteCost_(route);

      // Finally, add the travel time to the destination to the total cost
      total_cost_ += ComputeDistance_(current_stop.charger, destination) /
        route_planner_->speed();

      // Add the destination as the final stop on the route (don't bother
      // computing remaining range at destination)
      route.emplace_back(route_planner_->network().at(destination->name), 0, 0);
      return;
    }

    // Choose the next charger such that it minimizes the cost
    double key = candidates.begin()->first;
    Charger* next_charger = candidates.at(key);

    // If we've arrived at the current stop with less than the maximum range
    // (which should be true for every stop that's not the origin), determine
    // charge time to make it to the next stop    
    if ( current_stop.range < route_planner_->max_range() ) {
      current_stop.duration = 
        ComputeChargeTime_(current_stop, next_charger);

      // Update the total cost at the current stop
      UpdateRouteCost_(route);
    }

    // Compute the range remaining after arriving at the next stop
    double range_remaining = current_stop.range + 
      (current_stop.duration * current_stop.charger->rate) -
      ComputeDistance_(current_stop.charger, next_charger);

    // Add the next stop to the route and continue iteration. Note that the
    // charge duration at the next stop will be computed on the next iteration.
    route.emplace_back(next_charger, 0, range_remaining);
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

  void BruteForce::UpdateRouteCost_(const std::vector<Stop>& route) {
    // Get the current and previous stops
    const Stop& current = route.back();
    const Stop& previous = route.rbegin()[1];

    // Add the travel time between the previous and current stops
    total_cost_ += ComputeDistance_(previous.charger, current.charger) /
      route_planner_->speed();

    // Add the time to charge at the current stop
    total_cost_ += current.duration;
  }

  /**
   * @brief The "brute force" algorithm cost function.
   * 
   * @param current 
   * @param candidate 
   * @return double 
   */
  double BruteForce::ComputeCost_(
    const Charger* const current, const Charger* const candidate) const 
  {
    // Define the cost
    double cost{0};
    
    switch ( type_ )
    {
      case CostFcnType::MINIMIZE_DIST_REMAINING:
      {
        // The "cost" is the distance from the candidate charger to the
        // destination charger.
        cost = ComputeDistance_(candidate, route_planner_->destination());
        break;
      }
      
      case CostFcnType::MINIMIZE_TIME_REMAINING:
      {
        // Compute distances
        double candidate_to_destination = 
          ComputeDistance_(candidate, route_planner_->destination());
        double current_to_candidate = ComputeDistance_(current, candidate);

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
        throw std::invalid_argument("Ivalid cost fucntion type.");
        break;
    }
    return cost;
  }
} // end namespace supercharger