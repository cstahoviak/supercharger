/**
 * @file djikstras.cpp
 * @author Carl Stahoviak
 * @brief Dijkstra's algorithm implementation.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/logging.h"

#include <algorithm>
#include <queue>

namespace supercharger::algorithm
{
  /**
   * @brief Plans a route between the origin and the destination using
   * Dijkstra's algorithm.
   * 
   * @param origin The origin node.
   * @param destination The destination node.
   * @param max_range [km] The maximum range of the vehicle.
   * @param speed [km/hr] The vehicle's constant velocity.
   * @return PlannerResult The planner result.
   */
  PlannerResult Dijkstras::PlanRoute(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    // Define a lambda function for priority queue comparison.
    auto compare = [](
      const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs)
    {
      return lhs->cost > rhs->cost;
    };

    // Create the unvisited set as a priority queue.
    std::priority_queue<
      std::shared_ptr<Node>,
      std::vector<std::shared_ptr<Node>>,
      decltype(compare)> unvisited(compare);

    // Update the origin node's data and add it to the unvisited set.
    std::shared_ptr<Node>& origin_node = nodes_.at(origin);
    origin_node->arrival_range = max_range;
    origin_node->cost = 0;
    unvisited.push(origin_node);

    while ( !unvisited.empty() ) {
      // Get the next node (it has the smallest known distance from the origin)
      std::shared_ptr<Node> current_node = unvisited.top();
      unvisited.pop();

      // Skip the current node if it's already been visited.
      if ( current_node->visited ) {
        DEBUG("Already visited " << current_node->name() << ". Skipping.");
        continue;
      }

      // Mark the current node as visited prior to getting neighbors.
      current_node->visited = true;

      // If the current node is the destination node, we're done!
      if ( current_node->name() == destination ) {
        DEBUG("Final route cost: " << current_node->cost << " hrs.");
        return { ConstructFinalRoute_(*current_node), current_node->cost,
          max_range, speed };
      }

      // For the current node, consider all of its unvisited neighbors and
      // update their cost through the current node.
      double cost{0};
      for ( std::shared_ptr<Node>& neighbor : GetNeighbors_(*current_node, max_range) ) {
        // Compute the cost to get to the neighbor through the current node.
        // TODO: Is dereferencing here the right choice? Would it be better to
        // pass the shared pointer itself? Or a reference to the shared pointer
        // so as to not increase the reference count?
        cost = ComputeCost(*current_node, *neighbor, speed);

        if ( cost < neighbor->cost ) {
          // If the cost to the neighbor node through the current node is less
          // than the neighbor's current cost from the origin, update the
          // neighbor's cost and assign the current node as the neighbor's
          // parent.
          neighbor->cost = cost;
          neighbor->parent(current_node);

          // TODO: Could store charging durations and departure ranges as an
          // unordered map for each node. Then we wouldn't have to re-compute
          // these values during ConstructFinalRoute_().

          // Compute the charge time at the current node to reach the neighbor.
          double duration = ComputeChargeTime_(*current_node, *neighbor);

          // Compute the departure range at the current node.
          double departure_range = current_node->arrival_range + 
            duration * current_node->charger().rate;

          // Updating the arrival range at the neighbor node ensures that
          // the charge duration is calculated properly when the neighbor node
          // is considered as the "current" node.
          neighbor->arrival_range = departure_range - 
            math::distance(*current_node, *neighbor);

          // Add the neighbor to the unvisited set.
          // NOTE: It's likely that nodes that are already in the queue will be
          // added again by this line - that's okay.
          unvisited.emplace(neighbor);
        }
      }
    }

    // 7. Once the above loop exits, every reachable node will contain the
    // shortest distance from itself to the start node.

    // If we've gotten here, no path was found!
    INFO("No path found!");
    return {};
  }

  /**
   * @brief The Dijkstra's const function computes the cost at the neighbor node
   * as the sum of the cost at the current node + the charge time required to
   * reach the neighbor node from the current node + the travel time from the
   * current node to the neighbor.
   * 
   * Note that the cost associated with the neighbor node has no term that
   * accounts for the charging rate at the neighbor node. The cost function does
   * NOT include the charging rate or time at the neighbor node because we have
   * no information about the node that may come after the neighbor.
   * 
   * @param current The current Node.
   * @param neighbor  The neighbor Node.
   * @param speed The vehicle's constant velocity.
   * @return double The cost to the neighbor node through the current node.
   */
  double Dijkstras::ComputeCost(
    const Node& current, const Node& neighbor, double speed) const
  {
    return current.cost + ComputeChargeTime_(current, neighbor) +
      math::distance(current, neighbor) / speed;

    // TODO: Treat the cost function as a callable object.
    // return cost_f(current, neighbor, speed, cost_data_);
  }

  /**
   * @brief Returns all unvisited neighbors of the current node.
   * 
   * NOTE: Originally, I wanted the return type to be std::vector<Node* const>,
   * but this isn't possible. The vector is probably the only container that
   * requires the elements to be "copy assignable". This is because the elements
   * are guaranteed to be stored contiguously in memory. So if you exceed the
   * capacity, a new chunk has to be allocated and elements re-assigned. You
   * cannot do this with const elements.
   * 
   * @param current The current node.
   * @return A vector of neighbors as shared Node pointers.
   */
  std::vector<std::shared_ptr<Node>> Dijkstras::GetNeighbors_(
    const Node& current, double max_range)
  {
    std::vector<std::shared_ptr<Node>> neighbors;
    double current_to_neighbor{0};

    for ( const auto& [name, node] : nodes_ ) {
      current_to_neighbor = math::distance(current, *node);
      if ( current_to_neighbor <= max_range && !node->visited )
      {
        neighbors.push_back(node);
      }
    }

    return neighbors;
  }

  /**
   * @brief Constructs the final route by iterating over the Node's parents as a
   * linked list.
   * 
   * @return std::vector<Node> The final route from the origin node to the
   * destination node.
   */
  std::vector<Node> Dijkstras::ConstructFinalRoute_(const Node& final) {
    // Create the route and add the final node.
    std::vector<Node> route;
    route.push_back(final);

    // Iterate over the parents of each node to construct the complete path.
    std::shared_ptr<const Node> current_node = final.shared_from_this();
    while ( std::shared_ptr<const Node> parent = current_node->parent().lock() )
    {
      DEBUG(current_node->name() << " cost: " << current_node->cost << " hrs");
      
      // Add the parent to the route.
      // NOTE: Below we will update the charging durations for each node, but 
      // the cost previously computed for this node will not change because it
      // represents the minimum time required to reach this node, and thus we
      // can copy the cost when making a copy of the Node.
      route.push_back(*parent);
      current_node.swap(parent);
    }

    // Reverse the order to construct the final route.
    std::reverse(route.begin(), route.end());

    // We need to update the charging times for each node along the route
    // because the charging durations computed as part of Dijkstra's algorithm
    // are incorrect. The charge times represent the time required to charge at
    // a given node... (I need to think more about what's actually happening
    // during the Dijkstra's route planning process).
    for ( auto iter = route.begin(); iter != route.end() - 1; ++iter) {
      Node& current = *iter;
      Node& next = *(iter + 1);

      // Compute the charge time at the current node to reach the neighbor.
      current.duration = ComputeChargeTime_(current, next);
      DEBUG(current.name() << " updated charge time: " << current.duration << 
        " hrs");

      // Compute the departure range at the current node.
      current.departure_range = ComputeDepartureRange_(current);

      // Compute the arrival range at the neighbor node (this may be
      // unnecessary because it was computed during PlanRoute).
      next.arrival_range = ComputeArrivalRange_(current, next);
    }

    return route;
  }
} // end namespace supercharger::algorithm