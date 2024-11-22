/**
 * @file types.cpp
 * @author Carl Stahoviak
 * @brief Custom types for the Supercharger project.
 * @version 0.1
 * @date 2024-11-21
 * 
 * @copyright Copyright (c) 2024
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
      // TODO: Can this the simplified (the if statement removed) and replaced
      // by a single emplace_back() call that is "polymorphic aware"?
      if ( auto dijkstras_node = std::dynamic_pointer_cast<DijkstrasNode>(node) )
      {
        route.emplace_back(std::make_shared<DijkstrasNode>(*dijkstras_node));
      }
      else {
        route.emplace_back(std::make_shared<Node>(*node));
      }
    }
  }

  // Prior to writing the Move Assignment operator, code duplication within the
  // Copy Assignment operator (from the Copy Constructor) was required to make
  // the Copy Assignment operator work.
  // PlannerResult& PlannerResult::operator=(const PlannerResult& other) {
  //   // Directly copy the data stored via primitive types.
  //   cost = other.cost;
  //   max_range = other.max_range;
  //   speed = other.speed;

  //   // Create a new node, managed by a new shared_ptr.
  //   for ( const std::shared_ptr<Node>& node : other.route ) {
  //     if ( auto dijkstras_node = std::dynamic_pointer_cast<DijkstrasNode>(node) )
  //     {
  //       route.emplace_back(std::make_shared<DijkstrasNode>(*dijkstras_node));
  //     }
  //     else {
  //       route.emplace_back(std::make_shared<Node>(*node));
  //     }
  //   }
  //   return *this;
  // }

  PlannerResult& PlannerResult::operator=(const PlannerResult& other) {
    // Implement the copy assignment operator via the copy-and-swap idiom.
    // 1. Use the copy contructor to create a temporary object.
    // 2. Swap the data members of this object with the tempory object. Using
    // std::move requires implementing the move assignment operator.

    PlannerResult temp(other);
    std::swap(*this, temp);
    return *this;
  }

  PlannerResult& PlannerResult::operator=(PlannerResult&& other) {
    // Directly copy the data stored via primitive types.
    cost = other.cost;
    max_range = other.max_range;
    speed = other.speed;

    for ( const std::shared_ptr<Node>& node : other.route ) {
      // Acquire owndership of the shared_ptr from the moved-from object.
      route.push_back(std::move(node));
    }

    return *this;
  }
}