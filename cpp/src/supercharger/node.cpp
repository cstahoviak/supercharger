/**
 * @file node.cpp
 * @author Carl Stahoviak
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
  /**
   * @brief Resets the Node's attributes to their original values.
   */
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

  /**
   * @brief Formats the final route output to comply with the format expected
   * by the provided "checker" executables.
   * 
   * @param stream 
   * @param route 
   * @return std::ostream& 
   */
  std::ostream& operator<<(std::ostream& stream, const std::vector<Node>& route)
  {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const Node& node : route ) {
      stream << node;
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  std::ostream& operator<<(
    std::ostream& stream, const std::vector<std::shared_ptr<Node>>& route)
  {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const std::shared_ptr<Node>& node : route ) {
      stream << *node;
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }
} // end namespace supercharger