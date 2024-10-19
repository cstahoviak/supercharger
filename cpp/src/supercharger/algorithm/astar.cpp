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
#include "supercharger/algorithm/astar.h"

namespace supercharger::algorithm
{
  PlannerResult AStar::PlanRoute(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    return {};
  }

  double AStar::ComputeCost(
    const Node& current,
    const Node& neighbor,
    double speed) const 
  {
    return 0;
  }

  std::vector<Node> AStar::ConstructFinalRoute_(const Node& final)
  {
    return {};
  }
} // end namespace supercharger