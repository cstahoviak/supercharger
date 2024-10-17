#pragma once
/**
 * @file dijkstras.h
 * @author Carl Stahoviak
 * @brief Cost functions associated with Dijkstra's planner.
 * @version 0.1
 * @date 2024-10-17
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/node.h"

namespace supercharger::algorithm::cost
{
  /**
   * @brief Implements a "simple" cost function for Dijkstra's algorithm.
   * 
   * @return double 
   */
  double SimpleCost(const Node&, const Node&, double, void*);

  /**
   * @brief Implements an "optimized" cost function for Dijkstra's algorithm.
   * The optimized cost function uses a constrained optimization scheme to...
   * 
   * @return double 
   */
  double OptimizedCost(const Node&, const Node&, double, void*);
} // end namespace supercharger::algorithm::cost
