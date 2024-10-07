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
      // NaiveOptimizer() = default;
      PlannerResult Optimize(const PlannerResult&) const override;
  };
} // end namespace supercharger