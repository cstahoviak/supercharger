/**
 * @file astar.cpp
 * @author Carl Stahoviak
 * @brief Implements the A* path planning algorithm.
 * @version 0.1
 * @date 2024-08-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/astar.h"

namespace supercharger
{
  PlannerResult AStar::PlanRoute(
    const std::string& origin, const std::string& destination)
  {
    return {};
  }

  double AStar::ComputeCost(
    const Node* const current, const Node* const neighbor) const
  {
    return 0;
  }

  std::vector<Node> AStar::ConstructFinalRoute_(const Node* const final) {
    return {};
  }
} // end namespace supercharger