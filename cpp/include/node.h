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
    Node(Charger* charger) : charger(charger) {};

    // Store the charger associated with this node.
    Charger* charger{nullptr};

    // The length of time charging at this node (hrs).
    double duration{0};
    // The range of the vehicle upon arriving at this node.
    double arrival_range{0};
    // The post-charging range of the vehicle when departing this node.
    double departure_range{0}; 

    // The following were added for Dijkstra's implementation.
    // True if the node has been visited.
    bool visited{false};
    // The cost from this Node to the route origin. For Dijkstra's algorithm,
    // the cost is the total driving time plus total charging time upon arriving
    // at this node. It does NOT include the charging time at this node.
    double cost{std::numeric_limits<double>::max()};
    // Store the previous node on the route.
    Node* parent{nullptr};

    // Getters
    const std::string& name() const { 
      return ( charger ) ? charger->name : default_name_; }

  private:
    std::string default_name_{"NULL"};
  };

  std::ostream& operator<<(std::ostream&, const Node&);
  std::ostream& operator<<(std::ostream&, const Node* const);
} // end namespace supercharger