#pragma once
/**
 * @file stop.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "network.h"

namespace supercharger
{
  /**
   * @brief Represents a single stop along the route.
   * 
   */
  struct Stop {
    // Must define a constructor to make use of "emplace_back" like functions
    Stop(Charger* charger, double duration, double range, Stop* parent) :
      charger(charger), duration(duration), range(range), parent(parent) {};

    // Store the current charger
    Charger* charger;
    // The length of time charging (hrs)
    double duration{0};
    // The current range or the vehicle after arriving at the stop
    double range{0};
    // Store the previous stop on the route
    Stop* parent;
  };
} // end namespace supercharger