/**
 * @file types.cpp
 * @author Carl Stahoviak
 * @brief Custom types for the Supercharger project.
 * @version 0.1
 * @date 2024-11-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/types.h"


namespace supercharger
{
  PlannerResult::PlannerResult(
    std::vector<std::shared_ptr<Node>> route,
    double cost,
    double max_range,
    double speed) :
      route(std::move(route)), cost(cost), max_range(max_range), speed(speed) {}

  PlannerResult::PlannerResult(const PlannerResult& other) :
    cost(other.cost), max_range(other.max_range), speed(other.speed)
  {
    // Create a new node, managed by a new shared_ptr.
    for ( const std::shared_ptr<Node>& node : other.route ) {
      if ( auto dijkstras_node = std::dynamic_pointer_cast<DijkstrasNode>(node) )
      {
        route.emplace_back(std::make_shared<DijkstrasNode>(*dijkstras_node));
      }
      else {
        route.emplace_back(std::make_shared<Node>(*node));
      }
    }
  }

  PlannerResult& PlannerResult::operator=(const PlannerResult& other) {
    // Directly copy the data stored via primitive types.
    cost = other.cost;
    max_range = other.max_range;
    speed = other.speed;

    // Create a new node, managed by a new shared_ptr.
    for ( const std::shared_ptr<Node>& node : other.route ) {
      if ( auto dijkstras_node = std::dynamic_pointer_cast<DijkstrasNode>(node) )
      {
        route.emplace_back(std::make_shared<DijkstrasNode>(*dijkstras_node));
      }
      else {
        route.emplace_back(std::make_shared<Node>(*node));
      }
    }
    return *this;
  }

  // PlannerResult& PlannerResult::operator=(const PlannerResult& other) {
  //   // Implement the copy assignment operator via the copy-and-swap idiom.
  //   // 1. Use the copy contructor to create a temporary object.
  //   // 2. Swap the data members of this object with the tempory object.

  //   // TODO: This implementation segfaults. Must be related to storing the route
  //   // as a vector of shared_ptrs, or the fact that DijkstrasNode has a
  //   // weak_ptr member?
  //   PlannerResult temp(other);
  //   std::swap(*this, temp);
  //   return *this;
  // }
}