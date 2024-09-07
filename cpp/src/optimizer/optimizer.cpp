/**
 * @file optimizer.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "optimizer/optimizer.h"
#include "optimizer/naive.h"

namespace supercharger
{
  // NOTE: Do NOT repeat the 'static' keyword at the cpp file level.
  std::unique_ptr<Optimizer> Optimizer::GetOptimizer(OptimizerType type)
  {
    switch ( type )
    {
      case Optimizer::OptimizerType::NAIVE:
        // NOTE: There is an important difference between public and protected
        // inheritance when it comes to the compiler "being aware" that a
        // unique_ptr of the base class (the return type of this function) can
        // be initialized from a unique_ptr of the derived class: public
        // inheritance allows this, but protected inheritance does not. But why?
        return std::make_unique<NaiveOptimizer>();
      
      default:
        return std::unique_ptr<Optimizer>(nullptr);
    }
  }
} // end namespace supercharger