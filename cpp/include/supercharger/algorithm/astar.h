#pragma once
/**
 * @file astar.h
 * @author Carl Stahoviak
 * @brief The A* path planning algorithm.
 * @version 0.1
 * @date 2024-08-27
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/dijkstras.h"


namespace supercharger::algorithm
{
  /**
   * @brief Implements the A* search algorithm. A* makes sense in this scenario
   * because the total cost of the route is computed as a function of both the
   * distance (total time traveled) from the current node to the start node as 
   * well as a heuristic that, in our case, should be a function of the charge
   * rate.
   */
  class AStar : public Dijkstras
  {
    public:
      AStar(CostFunctionType cost_type) : Dijkstras(cost_type) {};
      AStar(DijkstrasCostFcnType cost_f) : Dijkstras(cost_f) {};

    protected:
      PlannerResult PlanRoute_(
        const std::string&,
        const std::string&,
        double,
        double) override;
      
      double ComputeCost_(
        const Node&,
        const Node&,
        double,
        double) const override;
      
      std::vector<std::shared_ptr<Node>> ConstructRoute_(const Node&) override;
  };
} // end namespace supercharger