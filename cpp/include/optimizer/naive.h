#pragma once
/**
 * @file naive.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "optimizer/optimizer.h"

namespace supercharger
{
  class NaiveOptimizer : public Optimizer
  {
    public:
      // NaiveOptimizer() = default;
      PlannerResult Optimize(const PlannerResult&) const override;
  };
} // end namespace supercharger