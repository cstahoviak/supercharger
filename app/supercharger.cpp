/**
 * @file supercharger.cpp
 * @author Carl Stahoviak
 * @brief The top-level Tesla Supercharger route planning application.
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "logging.h"
#include "planner.h"

#include <iostream>

using namespace supercharger;

int main(int argc, char** argv)
{
  if ( argc != 3 ) {
    std::cout << "Error: requires initial and final supercharger names."
      << std::endl;        
    return -1;
  }
  
  // Parse the initial and goal charger names
  const std::string initial_charger_name = argv[1];
  const std::string goal_charger_name = argv[2];

  // Create the Route Planner
  RoutePlanner planner(AlgoType::BRUTE_FORCE, CostFcnType::MINIMIZE_TIME_REMAINING);

  // Test to make sure the network getter works.
  auto& network = planner.network();

  // Plan the route with the provided cost function for the "brute force" algo
  std::vector<Stop> route = planner.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  LOG("\nFinal Route:");
  std::cout << route << std::endl;
}