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
    // Define a lambda function for priority queue comparison
    auto compare = [](const Stop* lhs, const Stop* rhs) {
      return lhs->cost < rhs->cost;
    };

    // 1. Create the unvisited set as a priority queue
    std::priority_queue<Stop*, std::vector<Stop*>, decltype(compare)> 
      unvisited(compare);

    // Update the origin node' data and add it to the unvisited set
    Stop& origin = nodes_.at(route.back().charger->name);
    origin.range = route_planner_->max_range();
    origin.cost = 0;
    unvisited.emplace(&origin);

    while ( !unvisited.empty() ) {
      // 3. The next stop is the one with the smallest known distance from the
      // origin. Mark the current stop as visited ans removed it from the
      // unvisited set.
      Stop* const current_stop = unvisited.top();
      current_stop->visited = true;
      unvisited.pop();

      // If the current node is the destination node, we're done!
      if ( current_stop->charger->name == route_planner_->destination()->name) {
        // We've arrived at the destination
        ConstructFinalRoute_(current_stop, route);
        return;
      }

      // 4. For the current node, consider all of its unvisited neighbors and
      // update their distance through the current node.
      double cost{0};
      for ( Stop* neighbor : GetNeighbors_(current_stop) ) {
        cost = ComputeDistance_(current_stop->charger, neighbor->charger);
        // If the distance to the neighbor node through the current node is less
        // than the neighbor's current distance from the origin, update the
        // neighbor's distance and assign the current node as the neighbor's
        // parent.
        if ( cost < neighbor->cost ) {
          neighbor->cost = cost;
          neighbor->parent = current_stop;
        }

        // Add the neighbor to the unvisited set.
        unvisited.emplace(neighbor);
      }

      // 5. Once we've considered all the unvisited neighbors of the current
      // node, mark the current node as visited and remove it from the unvisited
      // set.
    }

    // 7. Once the loop (steps 3-5) exits, every node will contain the shortest
    // distance from the start node.

    // If we've gotten here, no path was found!
    LOG("No path found!");
    return;
  }

  /**
   * @brief Returns all unvisited neighbors of the current node (stop).
   * 
   * NOTE: Cannot perfrom non-const iteration over a std::unordered_map via
   * structured bindings.
   * 
   * @param current 
   * @return std::vector<Stop* const> A vector of const pointers to Stops.
   */
  std::vector<Stop*> Dijkstras::GetNeighbors_(Stop* const current) {
    std::vector<Stop*> neighbors;
    double current_to_neighbor{0};
    for ( auto iter = nodes_.begin(); iter != nodes_.end(); ++iter ) {
      Stop& node = iter->second;
      current_to_neighbor = ComputeDistance_(current->charger, node.charger);
      if ( current_to_neighbor <= route_planner_->max_range() && !node.visited ) {
        neighbors.push_back(&node);
      }
    }

    return neighbors;
  }

  void Dijkstras::ConstructFinalRoute_(Stop* const final, std::vector<Stop>& route) {
    // Clear the route to prepare for reverse population and add the final stop
    route.clear();
    route.push_back(*final);

    // Iterate over the parents of each Stop to construct the complete path
    Stop* current_stop = final;
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
      // Update the range after arriving at the current stop
      prev_to_current = ComputeDistance_((iter - 1)->charger, iter->charger);
      iter->range = (iter - 1)->range - prev_to_current;

      // Update the total cost of the trip
      total_cost_ += prev_to_current / route_planner_->speed();

      // Compute the charge time for the current stop (skip the final stop)
      if ( iter != route.end() - 1 ) {
        iter->duration = ComputeChargeTime_(*iter, (iter + 1)->charger);
        total_cost_ += iter->duration;
      }
    }
  }
} // end namespace supercharger