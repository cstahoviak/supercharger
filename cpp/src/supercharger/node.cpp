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
  void SimpleNode::ResetNode() {
    duration = 0.0;
    arrival_range = 0.0;
    departure_range = 0.0;
  }

  void DijkstrasNode::ResetNode() {
    duration = 0.0;
    arrival_range = 0.0;
    departure_range = 0.0;
    visited = false;
    cost = std::numeric_limits<double>::max();
    parent_.reset();
  }

  std::ostream& operator<<(std::ostream& stream, const SimpleNode& node) {
    stream << node.name();
    if ( node.duration > 0 ) {
      stream << ", " << std::setprecision(7) << node.duration;
    }
    return stream;
  }

  std::ostream& operator<<(
    std::ostream& stream, const std::vector<SimpleNode>& route)
  {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const SimpleNode& node : route ) {
      stream << node;
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  std::ostream& operator<<(
    std::ostream& stream, const std::vector<std::shared_ptr<SimpleNode>>& route)
  {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const std::shared_ptr<SimpleNode>& node : route ) {
      stream << *node.get();
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }
} // end namespace supercharger