/**
 * @file brute_force.cpp
 * @author Carl Stahoviak
 * @brief Implements my "naive" route planner.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/naive.h"
#include "supercharger/logging.h"

#include <algorithm>
#include <map>
#include <stdexcept>


namespace supercharger::algorithm
{
  NaivePlanner::NaivePlanner(CostFunctionType type) : type_(type) {
    // Create a set of nodes from the charger network.
    for ( const Charger& charger : supercharger::NETWORK ) {
      const auto pair = nodes_.try_emplace(
        charger.name, std::make_shared<Node>(charger));
        if ( !pair.second ) {
          INFO("Charger with name '" << charger.name << "' already added to " <<
            "the node graph. Skipping.");
        }
    }
  }

  PlannerResult NaivePlanner::PlanRoute_(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    // Set the destination charger (required by the cost function).
    destination_ = nodes_.at(destination)->charger();

    // Initially, populate the route with the origin node.
    if ( route_.empty() ) {
      std::shared_ptr<Node>& origin_node = nodes_.at(origin);
      origin_node->arrival_range = max_range;
      route_.push_back(origin_node);
    }

    // Plan the route via recursion.
    PlanRouteRecursive_(origin, destination, max_range, speed);
    return { 
      ConstructRoute_(*nodes_.at(destination)), total_cost_, max_range, speed };
  }

  void NaivePlanner::PlanRouteRecursive_(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    // Get the current node and mark it as visited.
    std::shared_ptr<Node>& current = route_.back();
    DEBUG("Current route: " << route_);
    DEBUG("Arrival range at " << current->name() << ": " <<
      current->arrival_range);

    // Create a local alias for the destination node.
    const::std::shared_ptr<Node>& end_node = nodes_.at(destination);

    // Use a map to store the candidate nodes, i.e. the next possible nodes on
    // the route, sorted by math::distance to the destination.
    // TODO: Could use a priority queue instead? 
    std::map<double, std::shared_ptr<const Node>> candidates;
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
      current_to_candidate = math::distance(current, node);
      current_to_dest = math::distance(current, end_node);
      candidate_to_dest = math::distance(node, end_node);

      // If candidate node is within the maximum range of the vehicle, add the
      // candidate node to "reachable" set.
      if ( current_to_candidate <= max_range ) {
        is_reachable = true;
      }

      // If the candidate node is closer to the destination than the current
      // node, add the candidate node to the "closer to" set.
      if ( candidate_to_dest < current_to_dest ) {
        is_closer = true;
      }

      // If the node is reachable, and it's closer to the destination than
      // the current node, add it to the map of candidate nodes.
      if ( is_reachable && is_closer ) {
        // Compute the cost for the candidate charger.
        double cost = ComputeCost_(*current, *node, max_range, speed);

        // Add the charger to the map of candidate chargers.
        const auto& pair = candidates.try_emplace(cost, node);
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
    if ( std::find(candidate_names.begin(), candidate_names.end(), end_node->name())
      != candidate_names.end() )
    {
      // Compute the charge time to make it to the final destination
      current->duration = GetChargeTime(current, end_node);

      // Update the total route cost at the current node.
      UpdateRouteCost_(speed);

      // Finally, add the travel time to the destination to the total cost.
      total_cost_ += math::distance(current, end_node) / speed;

      // Construct the final route and return the planner result.
      return;
    }

    // Choose the next node such that it minimizes the cost.
    double key = candidates.begin()->first;
    std::shared_ptr<Node> next_node = 
      std::const_pointer_cast<Node>(candidates.at(key));

    // TODO: Maybe make this whole block its own function (UpdateCurrentNode_?)
    {
      // Compute the charge time and the departure range for the current node.
      current->duration = GetChargeTime(current, next_node);
      DEBUG("Charge duration at " << current->name() << ": " << 
        current->duration);

      // Update the total cost at the current node.
      UpdateRouteCost_(speed);
    }

    // Compute the range remaining after arriving at the next node.
    next_node->arrival_range = GetArrivalRange(current, next_node);

    // Add the next node to the route and continue iteration. Note that the
    // charge duration at the next node will be computed on the next iteration.
    route_.push_back(std::move(next_node));
    PlanRouteRecursive_(origin, destination, max_range, speed);

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
    //   double dist = math::distance(current_node.charger, charger);
    //   double charge_duration = (max_range_ - dist) / charger->rate;

    //   // Recursively call the brute force algorithm until the destination is
    //   // reached
    //   Node next_node = 
    //     { network_.at(charger->name), charge_duration, max_range_, &current_node };
    //   BruteForce_(next_node);
    // }

    return;
  }

  double NaivePlanner::ComputeCost_(
    const Node& current,
    const Node& candidate,
    double max_range,
    double speed) const
  {
    // Define the cost
    double cost{0};
    
    switch ( type_ )
    {
      case CostFunctionType::NAIVE_MINIMIZE_DIST_TO_NEXT:
      {
        cost = math::distance(current, candidate);
        break;
      }

      case CostFunctionType::NAIVE_MINIMIZE_DIST_REMAINING:
      {
        // The "cost" is the distance from the candidate charger to the
        // destination charger.
        cost = 
          math::distance(candidate.charger(), destination_);
        break;
      }
      
      case CostFunctionType::NAIVE_MINIMIZE_TIME_REMAINING:
      {
        // Compute distances
        double candidate_to_destination = 
          math::distance(candidate.charger(), destination_);
        double current_to_candidate = math::distance(current, candidate);

        // Compute times
        double time_to_destination = candidate_to_destination / speed;
        double time_to_charge = current_to_candidate / candidate.charger().rate;

        // The cost is the total time to drive the remaining distance between
        // the candidate charger and the destination + the time to fully charge
        // at the candidate charger.
        cost = (weight_time_to_destination_ * time_to_destination) + 
          (weight_time_to_charge_ * time_to_charge);
        break;
      }
      
      default:
        throw std::invalid_argument("Invalid cost fucntion type.");
    }
    return cost;
  }

  std::vector<std::shared_ptr<Node>> NaivePlanner::ConstructRoute_(
    const Node& final)
  {
    // Construct the route.
    std::vector<std::shared_ptr<Node>> route;
    for ( const std::shared_ptr<const Node>& node : route_ ) {
      // Construct a new node that is not a member of the node graph.
      route.emplace_back(std::make_shared<Node>(*node));
    }

    // Add the final Node.
    route.emplace_back(std::make_shared<Node>(final));

    return route;
  }

  void NaivePlanner::UpdateRouteCost_(double speed) {
    // Cost update can only take place for the second node onward.
    if ( route_.size() > 1 ) {
      // Get the current and previous nodes
      const std::shared_ptr<Node>& current = route_.back();
      const std::shared_ptr<Node>& previous = route_.rbegin()[1];

      // Add the travel time between the previous and current nodes
      total_cost_ += math::distance(previous, current) / speed;

      // Add the time to charge at the current node
      total_cost_ += current->duration;
    }
  }
} // end namespace supercharger::algorithm