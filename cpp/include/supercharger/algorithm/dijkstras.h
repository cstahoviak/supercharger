#pragma once
/**
 * @file dijkstras.h
 * @author Carl Stahoviak
 * @brief Dijkstra's path planning algorithm.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"

#include <functional>


namespace supercharger::algorithm
{
  /**
   * @brief Implements Dijkstra's algorithm.
   */
  class Dijkstras : public Planner
  {
    public:
      using DijkstrasCostFcnType =
        std::function<double(
          const DijkstrasNode&, const DijkstrasNode&, double, double)>;

      // Constructors.
      Dijkstras(CostFunctionType cost_type);
      Dijkstras(DijkstrasCostFcnType cost_f);

    protected:
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
      PlannerResult PlanRoute_(
        const std::string&,
        const std::string&,
        double,
        double) override;
      
      /**
       * @brief Dijkstra's algorithm cost function. Can be one of
       * Planner::CosFunctionType::DIJKSTRAS_SIMPLE or 
       * Planner::CosFunctionType::DIJKSTRAS_OPTIMIZED, or a custom cost
       * function provided as a callable object.
       * 
       * @param current The current node.
       * @param candidate The candidate "next" node.
       * @param max_range [km] The vehicle's max range.
       * @param speed [km/hr] The vehicle's constant velocity.
       * @return double The cost to reach the candidate node from the current
       * node.
       */
      double ComputeCost_(
          const Node&,
          const Node&,
          double,
          double) const override;
      
      /**
       * @brief Constructs the final route by iterating over the Node's parents
       * as a linked list.
       * 
       * @param final The final node in the route.
       * @return std::vector<Node> The final route from the origin node to the
       * destination node.
       */
      std::vector<std::shared_ptr<Node>> ConstructRoute_(const Node&) override;

    private:
      /**
       * @brief Creates the node graph for Dijkstra's algorithm.
       */
      void CreateNodeGraph_();

      /**
       * @brief Returns all unvisited neighbors of the current node.
       * 
       * @param current The current node.
       * @param max_range [km] The maximum range of the vehicle.
       * @return A vector of neighbors as shared Node pointers.
       */
      std::vector<std::shared_ptr<DijkstrasNode>> GetNeighbors_(
        const std::shared_ptr<const DijkstrasNode>&, double);
      
      DijkstrasCostFcnType cost_f;
  };

  /**
   * @brief Constructs a route given the final node of the route. 
   * 
   * @param final The final node in the route. Assumes that the full route can
   * be constructed via a linked list from the Node's 'parent' member.
   * @return std::vector<Node> The full route.
   */
  std::vector<std::shared_ptr<Node>> ConstructRoute(const DijkstrasNode&);

  /**
   * @brief The Dijkstra's "simple" cost function computes the cost at the
   * neighbor node as the sum of the cost at the current node + the charge time
   * required to reach the neighbor node from the current node + the travel time
   * from the current node to the neighbor.
   * 
   * Note that the cost associated with the neighbor node has no term that
   * accounts for the charging rate at the neighbor node. The cost function does
   * NOT include the charging rate or time at the neighbor node because we have
   * no information about the node that may come after the neighbor.
   * 
   * @param current The current Node.
   * @param neighbor  The neighbor Node.
   * @param max_range The vehicle's maximum range [km].
   * @param speed The vehicle's constant velocity [km/hr].
   * @return double The cost to the neighbor node through the current node.
   */
  double SimpleCost(const DijkstrasNode&, const DijkstrasNode&, double, double);

  /**
   * @brief Implements an "optimized" cost function for Dijkstra's algorithm.
   * The optimized cost function uses a constrained optimization scheme to...
   * 
   * @param current The current Node.
   * @param neighbor  The neighbor Node.
   * @param max_range The vehicle's maximum range [km].
   * @param speed The vehicle's constant velocity [km/hr].
   * @return double The cost to the neighbor node through the current node.
   */
  double OptimizedCost(const DijkstrasNode&, const DijkstrasNode&, double, double);
} // end namespace supercharger::algorithm