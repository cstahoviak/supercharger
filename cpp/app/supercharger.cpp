/**
 * @file supercharger.cpp
 * @author Carl Stahoviak
 * @brief The top-level Tesla Supercharger route planning application. The
 * output of this application is a single string in the format required by the
 * 'checker_linux' application used for route validation. The required output
 * format looks like:
 * 
 * <origin>, <node-2>, <charging-time-2>, <node-3>, <charging-time-2>, ...
 * <node-n>, <charging-time-n>, <destination>
 * 
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/supercharger.h"

#include <glog/logging.h>

#include <iostream>

using namespace supercharger;

int main(int argc, char** argv)
{
  if ( argc != 3 ) {
    std::cout << "Error: requires initial and final supercharger names."
      << std::endl;        
    return -1;
  }

  // Add glog
  google::InitGoogleLogging(argv[0]); 
  
  // Parse the initial and goal charger names
  const std::string initial_charger_name = argv[1];
  const std::string goal_charger_name = argv[2];

  // Create the Route Planner
  Supercharger app(AlgoType::DIJKSTRAS, OptimizerType::NLOPT);

  // Set the vehicle's speed and max range
  app.max_range() = 320;
  app.speed() = 105;

  // Plan the route
  PlannerResult result = app.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );

  // Output the result
  std::cout << result.route << std::endl;
}