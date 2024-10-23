/**
 * @file optimizer.cpp
 * @author Carl Stahoviak
 * @brief Defines the Optimizer interface functions.
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/optimize/naive.h"
#include "supercharger/optimize/nlopt.h"


namespace supercharger::optimize
{
  // NOTE: Do not repeat the 'static' keyword at the cpp file level.
  std::unique_ptr<Optimizer> Optimizer::GetOptimizer(OptimizerType type)
  {
    switch ( type )
    {
      case Optimizer::OptimizerType::NAIVE:
        return std::make_unique<NaiveOptimizer>();

      case Optimizer::OptimizerType::NLOPT:
        return std::make_unique<NLOptimizer>();
      
      default:
        return std::unique_ptr<Optimizer>(nullptr);
    }
  }
} // end namespace supercharger::optimize