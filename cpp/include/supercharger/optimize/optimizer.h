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
#include "supercharger/algorithm/algorithm.h"

namespace supercharger::optimize
{
  using PlannerResult = algorithm::PlannerResult;

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

      // NOTE: We don't require a default ctor to use std::make_unique()
      // Optimizer() = default;
      static std::unique_ptr<Optimizer> GetOptimizer(OptimizerType);
      
      // TODO: Consider passing a more limited scope of data to Optimize bc
      // the info contained by a complete PlannerResult contains more info than
      // is necessary to perfom the optimization. Byt passing a PlannerResult,
      // we couple the idea of a "Node" to the Optimizer. We want the Optimizer
      // to be agnostic to any given definition of a Node (so long as that Node
      // contains charging rates an durations).
      virtual PlannerResult Optimize(const PlannerResult&) const = 0;
  };
} // end namespace supercharger