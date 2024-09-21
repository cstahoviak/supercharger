#pragma once
/**
 * @file nloptimizer.h
 * @author Carl Stahoviak
 * @brief 
 * @version 0.1
 * @date 2024-09-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "optimizer/optimizer.h"

namespace supercharger
{
  class NLOptimizer : public Optimizer
  {
    public:
      struct ConstraintData
      {
        std::vector<double> distances;
        std::vector<double> rates;
        double init_arrival_range{0};
        double max_range{0};
      };

      PlannerResult Optimize(const PlannerResult&) const override;
  };
} // end namespace supercharger