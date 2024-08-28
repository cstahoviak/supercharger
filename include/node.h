#pragma once
/**
 * @file node.h
 * @author Carl Stahoviak
 * @brief The Node struct represents a single node along the route.
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
   * @brief Represents a single node along the route.
   */
  struct Node {
    // NOTE: Must define a ctor to make use of "emplace_back"-like functions.
    // NOTE: The Niave algorithm will use this constructor because it doesn't
    // seem like recursion and pass-by-reference play well with one another,
    // i.e. (I think) the route vector is copied between recursive calls to
    // PlanRoute, and thus the pointer to the parent Node ends up pointing to an
    // object that no longer exists.
    Node(Charger* charger, double duration, double range) : charger(charger),
      duration(duration), range(range) {};

    // For use with Dijkstra's algorithm.
    Node(Charger* charger) : charger(charger) {};

    // Store the charger associated with this node.
    Charger* charger{nullptr};

    // The length of time charging (hrs).
    double duration{0};
    // The current range or the vehicle after arriving at the node.
    double range{0};

    // The following were added for Dijkstra's implementation.
    // True if the node has been visited.
    bool visited{false};
    // The cost (distance for Dijkstra's) from this Node to the route origin.
    double cost{std::numeric_limits<double>::max()};
    // Store the previous node on the route.
    Node* parent{nullptr};
  };
} // end namespace supercharger