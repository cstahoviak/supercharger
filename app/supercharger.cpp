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

  // Plan the route with the provided cost function for the "brute force" algo
  std::vector<std::optional<Stop>> route =
    planner.PlanRoute(RoutePlanner::CostType::MINIMIZE_DIST_REMAINING);
  std::cout << "\nFinal Route:" << std::endl;
  std::cout << route << std::endl;
}