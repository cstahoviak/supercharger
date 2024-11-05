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

#include <stdexcept>
#include <sstream>


namespace supercharger::algorithm
{
  PlannerResult AStar::PlanRoute(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    std::ostringstream os;
    os << "The A* algorithm is not implemented. Please use Dijkstra's " <<
      "algorithm (supercharger::algorithm::AlgorithmType::DIJKSTRAS) instead.";
    throw std::runtime_error(os.str());
  }

  double AStar::ComputeCost(
    const Node& current,
    const Node& neighbor,
    double max_Range,
    double speed) const 
  {
    return 0;
  }

  std::vector<Node> AStar::ConstructRoute_(const Node& final)
  {
    return {};
  }
} // end namespace supercharger