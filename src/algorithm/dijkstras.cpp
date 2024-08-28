/**
 * @file djikstras.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/dijkstras.h"
#include "logging.h"
#include "planner.h"

#include <algorithm>
#include <queue>

namespace supercharger
{
  void Dijkstras::PlanRoute(std::vector<Stop>& route) {
    // Define a lambda function for priority queue comparison.
    auto compare = [](const Stop* const lhs, const Stop* const rhs) {
      return lhs->cost > rhs->cost;
    };

    // Create the unvisited set as a priority queue.
    std::priority_queue<Stop*, std::vector<Stop*>, decltype(compare)> 
      unvisited(compare);

    // Update the origin node's data and add it to the unvisited set.
    Stop& origin = nodes_.at(route.back().charger->name);
    origin.range = route_planner_->max_range();
    origin.cost = 0;
    unvisited.push(&origin);

    while ( !unvisited.empty() ) {
      // The next stop is the one with the smallest known distance from the
      // origin.
      Stop* const current_stop = unvisited.top();
      unvisited.pop();

      // Skip the current node if it's already been visited.
      if ( current_stop->visited ) {
        DEBUG("Already visited '" << current_stop->charger->name << 
          "'. Skipping.");
        continue;
      }

      // If the current node is the destination node, we're done!
      if ( current_stop->charger->name == route_planner_->destination()->name) {
        ConstructFinalRoute_(current_stop, route);
        return;
      }

      // Mark the current node as visited prior to getting neighbors.
      current_stop->visited = true;

      // For the current node, consider all of its unvisited neighbors and
      // update their cost through the current node.
      double cost{0};
      for ( Stop* const neighbor : GetNeighbors_(current_stop) ) {

        // Comoute the cost to get to the neighbor through the current node.
        cost = ComputeCost(*current_stop, neighbor->charger);

        // If the cost to the neighbor node through the current node is less
        // than the neighbor's current cost from the origin, update the
        // neighbor's cost and assign the current node as the neighbor's
        // parent.
        if ( cost < neighbor->cost ) {
          neighbor->cost = cost;
          neighbor->parent = current_stop;

          // Add the neighbor to the unvisited set.
          // NOTE: It's likely that nodes that are already in the queue will be
          // added again by this line - that's okay.
          unvisited.push(neighbor);
        }
      }
    }

    // 7. Once the above loop exits, every reachable node will contain the
    // shortest distance from itself to the start node.

    // If we've gotten here, no path was found!
    INFO("No path found!");
    return;
  }

  double Dijkstras::ComputeCost(const Stop& current, const Charger* const neighbor) const {
    return current.cost + ComputeChargeTime_(current, neighbor) +
      ComputeDistance_(current.charger, neighbor) / route_planner_->speed();
  }

  /**
   * @brief Returns all unvisited neighbors of the current node (stop).
   * 
   * NOTE: Originally, I wanted the return type to be std::vector<Stop* const>,
   * but this isn't possible. The vector is probably the only container that
   * requires the elements to be "copy assignable". This is because the elements
   * are guaranteed to be stored contiguously in memory. So if you exceed the
   * capacity, a new chunk has to be allocated and elements re-assigned. You
   * cannot do this with const elements.
   * 
   * @param current The current node (a const pointer to a const Stop).
   * @return std::vector<Stop* const> A vector of const pointers to Stops.
   */
  std::vector<Stop*> Dijkstras::GetNeighbors_(const Stop* const current) {
    std::vector<Stop*> neighbors;
    double current_to_neighbor{0};

    for ( auto& [name, node] : nodes_ ) {
      current_to_neighbor = ComputeDistance_(current->charger, node.charger);
      if ( current_to_neighbor <= route_planner_->max_range() && !node.visited ) {
        neighbors.push_back(&node);
      }
    }

    return neighbors;
  }

  void Dijkstras::ConstructFinalRoute_(
    const Stop* const final,
    std::vector<Stop>& route)
  {
    // Clear the route to prepare for reverse population and add the final stop
    route.clear();
    route.push_back(*final);

    // Iterate over the parents of each Stop to construct the complete path
    const Stop* current_stop = final;
    while ( current_stop->parent != nullptr ) {
      // Add the parent to the route
      route.push_back(*(current_stop->parent));
      current_stop = current_stop->parent;
    }

    // Reverse the order to construct the final route
    std::reverse(route.begin(), route.end());

    // Compute the charge time (duration) for each stop that's not the origin
    // or the destination.
    double prev_to_current{0};
    for ( auto iter = route.begin() + 1; iter != route.end(); ++iter) {
      const Stop& previous = *(iter - 1);
      Stop& current = *iter;
      const Stop& next = *(iter + 1);

      // Update the range after arriving at the current stop
      iter->range = ComputeRangeRemaining_(previous, current.charger);
      DEBUG("Range remaining at '" << current.charger->name << "': " <<
        iter->range);

      // Update the total cost of the trip
      prev_to_current = ComputeDistance_(previous.charger, current.charger);
      total_cost_ += prev_to_current / route_planner_->speed();

      // Compute the charge time at the current stop (skip the final stop)
      if ( current.charger->name != route_planner_->destination()->name ) {
        current.duration = ComputeChargeTime_(current, next.charger);
        DEBUG("Charging " << current.duration << " hrs at '" <<
          current.charger->name << "'" );
        total_cost_ += current.duration;
      }
    }
  }
} // end namespace supercharger