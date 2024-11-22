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
#include "supercharger/optimize/optimizer.h"
#include "supercharger/logging.h"

#include <algorithm>
#include <queue>
#include <sstream>


namespace supercharger::algorithm
{
  Dijkstras::Dijkstras(CostFunctionType cost_type) {
    switch ( cost_type )
    {
      case CostFunctionType::DIJKSTRAS_SIMPLE:
        cost_f = SimpleCost;
        break;

      case CostFunctionType::DIJKSTRAS_OPTIMIZED:
        cost_f = OptimizedCost;
        break;

      default:
        std::ostringstream os;
        os << "The cost function type '" << cost_type << "' is not " <<
        "compatible with the Dijkstras or A* Planners.";
        throw std::invalid_argument(os.str());
        break;
    }

    // Create the node graph.
    CreateNodeGraph_();
  }

  Dijkstras::Dijkstras(DijkstrasCostFcnType cost_f) : cost_f(std::move(cost_f))
  {
    // Create the node graph.
    CreateNodeGraph_();
  };

  PlannerResult Dijkstras::PlanRoute_(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    // Define a lambda function for priority queue comparison.
    auto compare = [](
      const std::shared_ptr<const DijkstrasNode>& lhs,
      const std::shared_ptr<const DijkstrasNode>& rhs)
    {
      return lhs->cost > rhs->cost;
    };

    // Create the unvisited set as a priority queue.
    std::priority_queue<
      std::shared_ptr<DijkstrasNode>,
      std::vector<std::shared_ptr<DijkstrasNode>>,
      decltype(compare)> unvisited(compare);

    // Update the origin node's data and add it to the unvisited set.
    const auto origin_node = 
      std::static_pointer_cast<DijkstrasNode>(nodes_.at(origin));
    origin_node->arrival_range = max_range;
    origin_node->cost = 0;
    unvisited.push(origin_node);

    while ( !unvisited.empty() ) {
      // Get the next node (it has the smallest known distance from the origin)
      const std::shared_ptr<DijkstrasNode> current_node = unvisited.top();
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
        return { ConstructRoute_(*current_node), current_node->cost,
          max_range, speed };
      }

      // For the current node, consider all of its unvisited neighbors and
      // update their cost through the current node.
      double cost{0};
      for ( const std::shared_ptr<DijkstrasNode>& neighbor : 
        GetNeighbors_(current_node, max_range) )
      {
        // Compute the cost to get to the neighbor through the current node.
        // TODO: Is dereferencing here the right choice? Would it be better to
        // pass the shared pointer itself? Or a reference to the shared pointer
        // so as to not increase the reference count?
        cost = ComputeCost_(*current_node, *neighbor, max_range, speed);

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
          double duration = GetChargeTime(current_node, neighbor);

          // Compute the departure range at the current node.
          double departure_range = current_node->arrival_range + 
            duration * current_node->charger().rate;

          // Updating the arrival range at the neighbor node ensures that
          // the charge duration is calculated properly when the neighbor node
          // is considered as the "current" node.
          neighbor->arrival_range = departure_range - 
            math::distance(current_node, neighbor);

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

  double Dijkstras::ComputeCost_(
    const Node& current,
    const Node& neighbor,
    double max_range,
    double speed) const
  {
    return cost_f(
      static_cast<const DijkstrasNode&>(current),
      static_cast<const DijkstrasNode&>(neighbor),
      max_range,
      speed);
  }
  
  void Dijkstras::CreateNodeGraph_() {
    // Create a set of nodes from the charger network.
    for ( const Charger& charger : supercharger::NETWORK ) {
      const auto pair = nodes_.try_emplace(
        charger.name, std::make_shared<DijkstrasNode>(charger));
        if ( !pair.second ) {
          INFO("Charger with name '" << charger.name << "' already added to " <<
            "the node graph. Skipping.");
        }
    }
  }

  std::vector<std::shared_ptr<DijkstrasNode>> Dijkstras::GetNeighbors_(
    const std::shared_ptr<const DijkstrasNode>& current, double max_range)
  {
    std::vector<std::shared_ptr<DijkstrasNode>> neighbors;
    double current_to_neighbor{0};

    for ( const auto& [name, node] : nodes_ ) {
      // Treat the node as a DijkstrasNode.
      const auto& dijkstras_node = 
        std::static_pointer_cast<DijkstrasNode>(node);

      current_to_neighbor = math::distance(current, dijkstras_node);
      if ( current_to_neighbor <= max_range && !dijkstras_node->visited )
      {
        neighbors.push_back(dijkstras_node);
      }
    }

    return neighbors;
  }

  std::vector<std::shared_ptr<Node>> Dijkstras::ConstructRoute_(
    const Node& final)
  {
    return ConstructRoute(static_cast<const DijkstrasNode&>(final));
  }

  std::vector<std::shared_ptr<Node>> ConstructRoute(const DijkstrasNode& final)
  {
    // Create the route and add the final node.
    std::vector<std::shared_ptr<Node>> route;
    route.emplace_back(std::make_shared<DijkstrasNode>(final));

    // Iterate over the parents of each node to construct the complete path.
    std::shared_ptr<const DijkstrasNode> current_node = final.shared_from_this();
    while ( std::shared_ptr<const DijkstrasNode> parent = 
      current_node->parent().lock() )
    {      
      // Add the parent to the route.
      // NOTE: Below we will update the charging durations for each node, but 
      // the cost previously computed for this node will not change because it
      // represents the minimum time required to reach this node, and thus we
      // can copy the cost when making a copy of the Node.
      route.emplace_back(std::make_shared<DijkstrasNode>(*parent));
      current_node.swap(parent);
    }

    // Reverse the order to construct the final route.
    std::reverse(route.begin(), route.end());

    // We need to update the charging times for each node along the route
    // because the charging durations computed as part of Dijkstra's algorithm
    // are incorrect. The charge times represent the time required to charge at
    // a given node... (I need to think more about what's actually happening
    // during the Dijkstra's route planning process).
    for ( auto iter = route.begin(); iter != route.end() - 1; ++iter ) {
      const std::shared_ptr<Node>& current = *iter;
      const std::shared_ptr<Node>& next = *(iter + 1);

      // Compute the charge time at the current node to reach the neighbor.
      current->duration = GetChargeTime(current, next);
      DEBUG(current->name() << " updated charge time: " << current->duration <<
        " hrs");

      // Compute the arrival range at the neighbor node.
      next->arrival_range = GetArrivalRange(current, next);
    }

    return route;
  }

  double SimpleCost(
    const DijkstrasNode& current,
    const DijkstrasNode& neighbor,
    double max_range,
    double speed)
  {
    return current.cost + GetChargeTime(current, neighbor) +
      math::distance(current, neighbor) / speed;
  }

  double OptimizedCost(
    const DijkstrasNode& current,
    const DijkstrasNode& neighbor,
    double max_range,
    double speed)
  {
    // Create the optimizer as a local static variable.
    using namespace supercharger::optimize;
    static const auto optimizer =
      Optimizer::GetOptimizer(Optimizer::OptimizerType::NLOPT);

    // Store the neighbor's current parent and substitute the current node as
    // the neighbor's parent.
    const std::shared_ptr<DijkstrasNode> original_parent = 
      neighbor.parent().lock();
    const_cast<DijkstrasNode&>(neighbor).parent(
      const_cast<DijkstrasNode&>(current).shared_from_this());

    // Create the route as if the neighbor node were the destination.
    std::vector<std::shared_ptr<Node>> route = ConstructRoute(neighbor);

    // Restore the neighbor's original parent.
    const_cast<DijkstrasNode&>(neighbor).parent(original_parent);

    if ( route.size() > 3 ) {
      // Optimize the route
      const PlannerResult result(route, 0.0, max_range, speed);
      const PlannerResult optimized = optimizer->Optimize(result);
      return optimized.cost;
    }
    else {
      return SimpleCost(current, neighbor, max_range, speed);
    }
  }
} // end namespace supercharger::algorithm