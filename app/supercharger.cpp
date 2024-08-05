/**
 * @file supercharger.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

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
    
  std::string initial_charger_name = argv[1];
  std::string goal_charger_name = argv[2];

  RoutePlanner planner(initial_charger_name, goal_charger_name);

  // Test to make sure the network getter works.
  auto& network = planner.network();

  // Plan the route
  std::vector<std::optional<Stop>> route = planner.PlanRoute();
  std::cout << "Final Route:\n";
  std::cout << route << std::endl;

  // Output the result
  // std::string output = planner.GenerateOutput(route);
  // std::cout << output << std::endl;
  return 0;
}