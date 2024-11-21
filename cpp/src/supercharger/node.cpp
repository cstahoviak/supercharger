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
#include "supercharger/math.h"
#include "supercharger/node.h"

#include <iomanip>

namespace supercharger
{  
  void Node::ResetNode_() {
    duration = 0.0;
    arrival_range = 0.0;
  }

  void DijkstrasNode::ResetNode_() {
    duration = 0.0;
    arrival_range = 0.0;
    visited = false;
    cost = std::numeric_limits<double>::max();
    parent_.reset();
  }

  std::ostream& operator<<(std::ostream& stream, const Node& node) {
    stream << node.name();
    if ( node.duration > 0 ) {
      stream << ", " << std::setprecision(7) << node.duration;
    }
    return stream;
  }

  // std::ostream& operator<<(std::ostream& stream, const DijkstrasNode& node) {
  //   return stream << static_cast<const Node&>(node);
  // }

  std::ostream& operator<<(
    std::ostream& stream, const std::vector<Node>& route)
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
      stream << *node.get();
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  double GetChargeTime(const Node& current, const Node& next)
  {
    // Compute the distance to the next charger.
    double current_to_next = math::distance(current, next);

    // Compute the charge time required to make it to the next charger.
    // NOTE: we're charging the car only enough to make it to the next node.
    double charge_time = 
      (current_to_next - current.arrival_range) / current.charger().rate;

    // If the charge time is negative, we have sufficient range without
    // additional charging to reach the next node.
    return std::max(0.0, charge_time);
  }

  double GetChargeTime(
    const std::shared_ptr<const Node>& current,
    const std::shared_ptr<const Node>& next)
  {
    return GetChargeTime(*current, *next);
  }

  double GetArrivalRange(const Node& current, const Node& next)
  {
    // The range remaining after arriving at the next node is the departure
    // range at the current node - the distance to the next charger.
    return current.departure_range() - math::distance(current, next);
  }

  double GetArrivalRange(
    const std::shared_ptr<const Node>& current,
    const std::shared_ptr<const Node>& next)
  {
    return GetArrivalRange(*current, *next);
  }
} // end namespace supercharger