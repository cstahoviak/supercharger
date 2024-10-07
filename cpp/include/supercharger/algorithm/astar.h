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
      AStar(RoutePlanner* rp) : Dijkstras(rp) {};
      PlannerResult PlanRoute(const std::string&, const std::string&) override;
      double ComputeCost(const Node&, const Node&) const override;

    protected:
      std::vector<Node> ConstructFinalRoute_(const Node&) override;
  };
} // end namespace supercharger