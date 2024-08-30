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

  // Use the "naive" planning algorithm
  if (true) {
    // Create the Route Planner
    RoutePlanner planner(
      AlgoType::NAIVE,
      CostFcnType::MINIMIZE_TIME_REMAINING
    );

    // Test to make sure the network getter works.
    const auto& network = planner.network();

    // Plan the route with chosen algorithm and cost function
    std::vector<Node> route = planner.PlanRoute(
      initial_charger_name,
      goal_charger_name
    );
    INFO("Nainve Planner Final Route (Cost: " << planner.cost() << " hrs)");
    std::cout << route << std::endl;
  }

  std::cout << "\n";

  // Dijkstra's Algorithm
  if (true) {
    // Create the Route Planner
    RoutePlanner planner(AlgoType::DIJKSTRAS);

    // Plan the route with chosen algorithm and cost function
    std::vector<Node> route = planner.PlanRoute(
      initial_charger_name,
      goal_charger_name
    );
    INFO("Dijkstra's Final Route (Cost: " << planner.cost() << " hrs)");
    std::cout << route << std::endl;

    std::vector<Node> optimized = planner.OptimizeRoute(route);
    INFO("Optimized Route (Cost: hrs)");
    std::cout << optimized << std::endl;
  }
}