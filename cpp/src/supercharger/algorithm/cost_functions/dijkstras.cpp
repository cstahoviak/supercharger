/**
 * @file dijkstras.cpp
 * @author Carl Stahoviak
 * @brief Cost functions associated with Dijkstra's planner.
 * @version 0.1
 * @date 2024-10-17
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/algorithm/cost_functions/dijkstras.h"

namespace supercharger::algorithm::cost
{
  double SimpleCost(
    const Node& current,
    const Node& neighbor,
    double speed,
    void* data)
  {
    // return current.cost + ComputeChargeTime(current, neighbor) +
    //   math::distance(current, neighbor) / speed;
    return 0;
  }

  double OptimizedCost(
    const Node& current,
    const Node& neighbor,
    double speed,
    void* data)
  {
    return 0;
  }
}