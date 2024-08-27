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
#include <unordered_set>

namespace supercharger
{
  void Dijkstras::PlanRoute(std::vector<Stop>& route) {
    // Define a lambda function for priority queue comparison
    auto compare = [](const Stop* const lhs, const Stop* const rhs) {
      return lhs->cost > rhs->cost;
    };

    // 1. Create the unvisited set as a priority queue
    std::priority_queue<Stop*, std::vector<Stop*>, decltype(compare)> 
      unvisited(compare);
    // for (auto& [name, node] : nodes_ ) {
    //   if ( node.charger->name == route.back().charger->name ) {
    //     DEBUG("Origin Stop Address: " << &node);
    //     node.range = route_planner_->max_range();
    //     node.cost = 0;
    //   }
    //   unvisited.push(&node);
    // }

    // Store the set of nodes in the priority queue
    std::unordered_set<int> unisted_ids;

    // Update the origin node's data
    Stop& origin = nodes_.at(route.back().charger->name);
    DEBUG("Origin Stop Address: " << &origin);
    origin.range = route_planner_->max_range();
    origin.cost = 0;
    unvisited.push(&origin);

    while ( !unvisited.empty() ) {
      DEBUG("\nPriority Queue Size: " << unvisited.size());

      // 3. The next stop is the one with the smallest known distance from the
      // origin. Mark the current stop as visited ans removed it from the
      // unvisited set.
      Stop* const current_stop = unvisited.top();
      DEBUG("Current Stop: " << current_stop->charger->name);
      unvisited.pop();
      if ( current_stop->visited ) {
        DEBUG("Already visited '" << current_stop->charger->name << 
          "'. Skipping.");
        continue;
      }

      // Mark the current node as visited prior to getting neighbors.
      current_stop->visited = true;

      // If the current node is the destination node, we're done!
      if ( current_stop->charger->name == route_planner_->destination()->name) {
        // We've arrived at the destination
        ConstructFinalRoute_(current_stop, route);
        return;
      }

      if ( current_stop->charger->name == "Madison_WI" ) {
        DEBUG("\nCurrent Stop: " << current_stop->charger->name);
        DEBUG("Previous cost to neighbors:");

        for ( Stop* neighbor : GetNeighbors_(current_stop) ) {
          DEBUG(neighbor->charger->name << ": " << neighbor->cost);
        }
      }

      // 4. For the current node, consider all of its unvisited neighbors and
      // update their distance through the current node.
      double cost{0};
      for ( Stop* neighbor : GetNeighbors_(current_stop) ) {
        // TODO: The cost update below is wrong.
        cost = current_stop->cost +
          ComputeDistance_(current_stop->charger, neighbor->charger);

        // If the distance to the neighbor node through the current node is less
        // than the neighbor's current distance from the origin, update the
        // neighbor's distance and assign the current node as the neighbor's
        // parent.
        if ( cost < neighbor->cost ) {
          neighbor->cost = cost;
          neighbor->parent = current_stop;

          // Add the neighbor to the unvisited set.
          // NOTE: It's likely that nodes that are already in the queue will be
          // added again by this line - that's okay.
          DEBUG("Adding neighbor '" << neighbor->charger->name << "'");
          unvisited.push(neighbor);
        }
      }

      if ( current_stop->charger->name == "Madison_WI" ) {
        DEBUG("\nUpdated cost to neighbors:");

        for ( Stop* neighbor : GetNeighbors_(current_stop) ) {
          DEBUG(neighbor->charger->name << ": " << neighbor->cost);
        }
      }

      // 5. Once we've considered all the unvisited neighbors of the current
      // node, mark the current node as visited and remove it from the unvisited
      // set.
    }

    // 7. Once the loop (steps 3-5) exits, every node will contain the shortest
    // distance from the start node.

    // If we've gotten here, no path was found!
    INFO("No path found!");
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
    DEBUG("Getting neighbors of: " << current->charger->name);
    std::vector<Stop*> neighbors;
    double current_to_neighbor{0};
    for ( auto& [name, node] : nodes_ ) {
      if ( node.visited ) {
        // DEBUG("Already visited: " << node.charger->name);
      }
      current_to_neighbor = ComputeDistance_(current->charger, node.charger);
      if ( current_to_neighbor <= route_planner_->max_range() && !node.visited ) {
        neighbors.push_back(&node);
      }
    }

    DEBUG("Neighbors (" << neighbors.size() << "):");
    for (const Stop* const stop : neighbors) {
      DEBUG(stop->charger->name);
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
      Stop& current = *iter;
      Stop& previous = *(iter - 1);

      // Update the range after arriving at the current stop
      iter->range = ComputeRangeRemaining_(previous, current.charger);
      DEBUG("Range remaining at '" << current.charger->name << "': " <<
        iter->range);

      // Update the total cost of the trip
      prev_to_current = ComputeDistance_(previous.charger, current.charger);
      total_cost_ += prev_to_current / route_planner_->speed();

      // Compute the charge time for the current stop (skip the final stop)
      if ( iter != route.end() - 1 ) {
        iter->duration = ComputeChargeTime_(*iter, (iter + 1)->charger);
        total_cost_ += iter->duration;
      }
    }
  }
} // end namespace supercharger