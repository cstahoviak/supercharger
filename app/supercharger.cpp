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

  // Use the Brute Force Algorithm
  if (true) {
    // Create the Route Planner
    RoutePlanner planner(
      AlgoType::BRUTE_FORCE,
      CostFcnType::MINIMIZE_TIME_REMAINING
    );

    // Test to make sure the network getter works.
    const auto& network = planner.network();

    // Plan the route with chosen algorithm and cost function
    std::vector<Stop> route = planner.PlanRoute(
      initial_charger_name,
      goal_charger_name
    );
    DEBUG("\n'Brute Force' Final Route (Cost: " << planner.cost() << " hrs)");
    std::cout << route << std::endl;
  }

  std::cout << "\n";

  // Dijkstra's Algorithm
  if (true) {
    // Create the Route Planner
    RoutePlanner planner(AlgoType::DIJKSTRAS);

    // Plan the route with chosen algorithm and cost function
    std::vector<Stop> route = planner.PlanRoute(
      initial_charger_name,
      goal_charger_name
    );
    DEBUG("\nDijkstra's Final Route (Cost: " << planner.cost() << " hrs)");
    std::cout << route << std::endl;
  }
}