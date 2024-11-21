#pragma once
/**
 * @file optimizer.h
 * @author Carl Stahoviak
 * @brief Defines the Optimizer base class.
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"
#include "supercharger/types.h"


namespace supercharger::optimize
{
  /**
   * @brief Defines the public interface for all optimization classes.
   */
  class Optimizer
  {
    public:
      enum class OptimizerType {
        NAIVE,
        NLOPT,
        NONE
      };

      /**
       * @brief Gets a unique_ptr to an Optimizer instance.
       * 
       * @param OptimizerType The optimizer type.
       * @return std::unique_ptr<Optimizer> 
       */
      static std::unique_ptr<Optimizer> GetOptimizer(OptimizerType);

      /**
       * @brief Optimizes the provided PlanbnerResult.
       * 
       * TODO: Consider passing a more limited scope of data to Optimize. We
       * only require charging rates and durations, and the distances between
       * nodes to optimize a route.
       * 
       * @param result The ouput of a Planner.
       * @return PlannerResult 
       */
      virtual PlannerResult Optimize(const PlannerResult&) const = 0;
  };
} // end namespace supercharger