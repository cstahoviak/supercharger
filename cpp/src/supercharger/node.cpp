/**
 * @file node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/node.h"

#include <iomanip>

namespace supercharger
{
  void Node::Reset() {
    duration = 0.0;
    arrival_range = 0.0;
    departure_range = 0.0;
    visited = false;
    cost = std::numeric_limits<double>::max();
    parent_.reset();
  }

  std::ostream& operator<<(std::ostream& stream, const Node& node) {
    return stream << std::addressof(node);
  }

  std::ostream& operator<<(std::ostream& stream, const Node* const node) {
    stream << node->name();
    if ( node->duration > 0 ) {
      stream << ", " << std::setprecision(7) << node->duration;
    }
    return stream;
  }
}