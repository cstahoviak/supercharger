#pragma once
/**
 * @file utils.h
 * @author Carl Stahoviak
 * @brief Generic utility functions.
 * @version 0.1
 * @date 2024-10-21
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/node.h"

#include <functional>

namespace supercharger
{  
  /**
   * @brief Wanted to use this as a place-holder for a null cost function, but I
   * wasn't able to get it to work.
   * 
   * Source: https://stackoverflow.com/questions/57522428/make-the-compiler-generate-an-empty-default-function-for-an-stdfunction
   */
  struct nullfunc_t
  {
    template<class F>
    operator std::function<F>() { return [](auto&&...){}; }
  } inline nullfunc;

  // NOTE: Must declare 'inline' to prevent "multiple definition" errors.
  inline std::function<double(const Node&, const Node&, double)> nullcostfunc = 
    [](auto&&...){ return 0; };
} // end namespace sueprcharger