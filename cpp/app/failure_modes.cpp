/**
 * @file failure_modes.cpp
 * @author Carl Stahoviak
 * @brief An exploration or currently known failure modes.
 * @version 0.1
 * @date 2024-10-17
 * 
 * @copyright Copyright (c) 2024
 */

#include "supercharger/supercharger.h"

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
  Supercharger app(AlgoType::DIJKSTRAS, OptimizerType::NLOPT);

  // Set a vehicle range that is not great enough to get to any neighboring nodes
  app.max_range() = 10;
  app.speed() = 105;

  // Plan the route
  PlannerResult result = app.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );

  // Output the result
  std::cout << result.route << std::endl;
}