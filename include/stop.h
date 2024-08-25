#pragma once
/**
 * @file stop.h
 * @author Carl Stahoviak
 * @brief The Stop struct represents a single node along the route.
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 */
#include "network.h"

#include <limits>

namespace supercharger
{
  /**
   * @brief Represents a single stop along the route.
   */
  struct Stop {
    // Must define a constructor to make use of "emplace_back"-like functions
    Stop(Charger* charger, double duration, double range, Stop* parent) :
      charger(charger), duration(duration), range(range), parent(parent) {};

    // NOTE: The Brute Force algorithm will use this constructor because it
    // doesn't seem like recursion and pass-by-reference play well with one
    // another, i.e. (I think) the route vector is copied between recursive
    // calls to plan route, and thus the pointer to the parent Stop ends up
    // pointing to an object that no longer exists.
    Stop(Charger* charger, double duration, double range) :
      Stop(charger, duration, range, nullptr) {};

    // For use with Dijkstra's algorithm
    Stop(Charger* charger, double range, double cost, bool visited) : 
      charger(charger), range(range), cost(cost), visited(visited) {};
    Stop(Charger* charger) : charger(charger) {};
    // Stop(Charger* charger) : Stop(charger, 0, 0, nullptr) {};

    // Store the charger associated with this stop
    Charger* charger{nullptr};

    // The length of time charging (hrs)
    double duration{0};
    // The current range or the vehicle after arriving at the stop
    double range{0};

    // The following were added for Dijstra's implementation

    // True if the node has been visited
    bool visited{false};
    // The cost (distance for Dijkstra's) from this Stop to the route origin
    double cost{std::numeric_limits<double>::max()};
    // Store the previous stop on the route
    Stop* parent{nullptr};
  };
} // end namespace supercharger