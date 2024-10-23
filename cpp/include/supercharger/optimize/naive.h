#pragma once
/**
 * @file naive.h
 * @author Carl Stahoviak
 * @brief The "naive" optimizer.
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/optimize/optimizer.h"

namespace supercharger::optimize
{
  class NaiveOptimizer : public Optimizer
  {
    public:
      /**
       * @brief Implements the "naive" optimization scheme. The "naive"
       * optimizer finds the node in the route with the max-rate charger (that
       * is not the first, last or second-to-last node) and minimizes the
       * route's total charge time by maximizing the charge time at the max-rate
       * node.
       * 
       * @param result An "unoptimized" planner result.
       * @return PlannerResult The optimized planner result.
       */
      PlannerResult Optimize(const PlannerResult&) const override;
  };
} // end namespace supercharger