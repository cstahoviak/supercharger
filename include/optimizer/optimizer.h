#pragma once
/**
 * @file optimizer.h
 * @author Carl Stahoviak
 * @brief Defines the Optimizer template class.
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "planner.h"

namespace supercharger
{
  /**
   * @brief Defines the public interface for all optimization classes.
   */
  class Optimizer
  {
    public:
      enum class OptimizerType {
        NAIVE
      };

      Optimizer() = default;
      static std::unique_ptr<Optimizer> GetOptimizer(OptimizerType&&);
      
      virtual PlannerResult Optimize(const PlannerResult&) const = 0;
  };
} // end namespace supercharger