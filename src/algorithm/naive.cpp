/**
 * @file brute_force.cpp
 * @author Carl Stahoviak
 * @brief Implements my "naive" route planner.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/naive.h"
#include "logging.h"
// Need to include planner.h here to avoid "pointer or reference to incomplete
// type is not allowed" errors related to using the route_planner_ pointer.
#include "planner.h"

#include <algorithm>
#include <map>
#include <stdexcept>


namespace supercharger
{
  /**
   * @brief The "naive" route planner.
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
  PlannerResult Naive::PlanRoute(
    const std::string& origin, const std::string& destination)
  {
    // Initially, populate the route with the origin node.
    if ( route_.empty() ) {
      Node& origin_node = nodes_.at(origin);
      origin_node.arrival_range = route_planner_->max_range();
      route_.push_back(&origin_node);
    }

    // Plan the route via recursion.
    PlanRouteRecursive_(origin, destination);
    return { ConstructFinalRoute_(&nodes_.at(destination)), total_cost_ };
  }

  void Naive::PlanRouteRecursive_(
    const std::string& origin, const std::string& destination)
  {
    // Get the current node and mark it as visited.
    Node* const current = route_.back();
    current->visited = true;
    DEBUG("Current route: " << route_);
    DEBUG("Arrival range at " << current->name() << ": " <<
      current->arrival_range);

    // Create a local alias for the destination node.
    const Node* const end_node = std::addressof(nodes_.at(destination));

    // Use a map to store the candidate nodes, i.e. the next possible nodes on
    // the route, sorted by distance to the destination.
    // TODO: Could use a priority queue instead? 
    std::map<double, const Node*> candidates;
    std::vector<std::string> candidate_names;
    bool is_reachable{false};
    bool is_closer{false};

    // Store relative distance measurements.
    double current_to_candidate{0};
    double current_to_dest{0};
    double candidate_to_dest{0};

    for ( const auto& [name, node] : nodes_ ) {
      is_reachable = false;
      is_closer = false;

      // Compute distances.
      current_to_candidate = compute_distance(current, std::addressof(node));
      current_to_dest = compute_distance(current, end_node);
      candidate_to_dest = compute_distance(std::addressof(node), end_node);

      // If candidate node is within the maximum range of the vehicle, add the
      // candidate node to "reachable" set.
      if ( current_to_candidate <= route_planner_->max_range() ) {
        is_reachable = true;
      }

      // If the candidate node is closer to the destination than the current
      // node, add the candidate node to the "closer to" set.
      if ( candidate_to_dest < current_to_dest ) {
        is_closer = true;
      }

      // If the node is reachable, and it's closer to the destination than
      // the current node, add it to map of candidate nodes.
      if ( is_reachable && is_closer ) {
        // Compute the cost for the candidate charger.
        double cost = ComputeCost(current, std::addressof(node));

        // Add the charger to the map of candidate chargers.
        const auto& pair = candidates.try_emplace(cost, std::addressof(node));
        candidate_names.push_back(name);
      }
    }

    // TODO: Compute the intersection of the "reachable" and "closer to" sets
    // std::unordered_set<Charger> candidate_chargers;
    // std::set_intersection(reachable.begin(), reachable.end(),
    //   closer_to.begin(), closer_to.end(),
    //   std::inserter(candidate_chargers, candidate_chargers.begin()));

    // If the destination is in the set of candidate nodes, we can go to the
    // destination on our remaining charge and we're done!
    if ( std::find(candidate_names.begin(), candidate_names.end(), end_node->name()) != candidate_names.end() ) {
      // Compute the charge time to make it to the final destination
      current->duration = ComputeChargeTime_(current, end_node);

      // Update the total route cost at the current node.
      UpdateRouteCost_();

      // Finally, add the travel time to the destination to the total cost.
      total_cost_ += compute_distance(current, end_node) /
        route_planner_->speed();

      // Construct the final route and return the planner result.
      return;
    }

    // Choose the next node such that it minimizes the cost.
    double key = candidates.begin()->first;
    Node* const next_node = const_cast<Node* const>(candidates.at(key));

    // TODO: Maybe make this whole block its own function (UpdateCurrentNode_?)
    {
      // Compute the charge time and the departure range for the current node.
      current->duration = ComputeChargeTime_(current, next_node);
      DEBUG("Charge duration at " << current->name() << ": " << 
        current->duration);

      // Update the departure range for the current node.
      current->departure_range = ComputeDepartureRange_(current);

      // Update the total cost at the current node.
      UpdateRouteCost_();
    }

    // Compute the range remaining after arriving at the next node.
    next_node->arrival_range = ComputeArrivalRange_(current, next_node);

    // Add the next node to the route and continue iteration. Note that the
    // charge duration at the next node will be computed on the next iteration.
    route_.emplace_back(next_node);
    PlanRouteRecursive_(origin, destination);

    // NOTE: My initial thought was to not choose a single "next charger", but
    // recursively call PlanRoute on ALL candidate chargers. Thus allowing the
    // search space to grow beyond a single charger at each iteration. I have no
    // idea if this is even feasible, or if it can be accomplished via
    // recursion. Below is the code I wrote to attempt that (it didn't work).
    
    // If the destination is not in the reachable set, for each Charger in the
    // reachable set, compute its reachable set. Repeat until the destination
    // charger is in the reachable set.
    // for ( const auto& [name, charger] : candidate_chargers ) {
    //   // Compute the charge time to return to max charge (Assume for now that 
    //   // the vehicle leaves the current node with a full charge)
    //   double dist = compute_distance(*(current_node.charger), *charger);
    //   double charge_duration = (max_range_ - dist) / charger->rate;

    //   // Recursively call the brute force algorithm until the destination is
    //   // reached
    //   Node next_node = 
    //     { network_.at(charger->name), charge_duration, max_range_, &current_node };
    //   BruteForce_(next_node);
    // }

    return;
  }

  std::vector<Node> Naive::ConstructFinalRoute_(const Node* const final) {
    // Construct the route.
    std::vector<Node> route;
    for ( const Node* const node : route_ ) {
      route.push_back(*node);
    }

    // Mark the final node as visited and and return the route.
    const_cast<Node* const>(final)->visited = true;
    route.push_back(*final);
    return route;
  }

  void Naive::UpdateRouteCost_() {
    // Cost update can only take place for the second node onward.
    if ( route_.size() > 1 ) {
      // Get the current and previous nodes
      const Node* const current = route_.back();
      const Node* const previous = route_.rbegin()[1];

      // Add the travel time between the previous and current nodes
      total_cost_ += compute_distance(previous, current) /
        route_planner_->speed();

      // Add the time to charge at the current node
      total_cost_ += current->duration;
    }
  }

  /**
   * @brief The "brute force" algorithm cost function.
   * 
   * @param current 
   * @param candidate 
   * @return double 
   */
  double Naive::ComputeCost(
    const Node* const current, const Node* const candidate) const 
  {
    // Define the cost
    double cost{0};
    
    switch ( type_ )
    {
      case CostFcnType::MINIMIZE_DIST_TO_NEXT:
      {
        // Effectively the same as Dijkstra's
        cost = compute_distance(current, candidate);
        break;
      }

      case CostFcnType::MINIMIZE_DIST_REMAINING:
      {
        // The "cost" is the distance from the candidate charger to the
        // destination charger.
        cost = compute_distance(candidate->charger,
          const_cast<const Charger* const>(route_planner_->destination()));
        break;
      }
      
      case CostFcnType::MINIMIZE_TIME_REMAINING:
      {
        // Compute distances
        double candidate_to_destination = compute_distance(candidate->charger,
          const_cast<const Charger* const>(route_planner_->destination()));
        double current_to_candidate = compute_distance(current, candidate);

        // Compute times
        double time_to_destination = 
          candidate_to_destination / route_planner_->speed();
        double time_to_charge = current_to_candidate / candidate->charger->rate;

        // The cost is the total time to drive the remaining distance between
        // the candidate charger and the destination + the time to fully charge
        // at the candidate charger.
        cost = weight_time_to_destination_ * time_to_destination + 
          weight_time_to_charge_ * time_to_charge;
        break;
      }
      
      default:
        throw std::invalid_argument("Invalid cost fucntion type.");
        break;
    }
    return cost;
  }
} // end namespace supercharger