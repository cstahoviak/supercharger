/**
 * @file algorithm_comparison.cpp
 * @author Carl Stahoviak
 * @brief A comparison of the planning and optimization algorithm currently
 * implemented.
 * @version 0.1
 * @date 2024-10-01
 * 
 * @copyright Copyright (c) 2024
 */
#include "logging.h"
#include "planner.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>

using namespace supercharger;

template<typename A, typename B>
std::pair<B, A> flip_pair(const std::pair<A, B>& p) {
    return std::pair<B,A>(p.second, p.first);
}

template<typename A, typename B>
std::multimap<B, A, std::greater<B>> flip_map(const std::map<A, B>& src) {
    std::multimap<B, A, std::greater<B>> dst;
    std::transform(
      src.begin(), src.end(), std::inserter(dst, dst.begin()), flip_pair<A, B>);
    return dst;
}

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

  // Store a map of planning algorithms and route costs
  std::map<std::string, double> algorithms;

  // Create the Route Planner
  RoutePlanner planner(
    AlgoType::NAIVE,
    CostFcnType::MINIMIZE_TIME_REMAINING
  );

  // Set the vehicle's speed and max range
  planner.max_range() = 320;
  planner.speed() = 105;

  // Test to make sure the network getter works.
  const auto& network = planner.network();

  // Plan the route via the "Naive" route planner
  PlannerResult result1 = planner.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  algorithms["NAIVE\t\t\t\t"] = result1.cost;

  // Plan the route via Dijkstra's algorithm.
  planner.SetPlanningAlgorithm(AlgoType::DIJKSTRAS);
  PlannerResult result2 = planner.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  algorithms["DIJKTRA'S\t\t\t"] = result2.cost;

  // Improve on Dijkstra's via the "naive" optimizer.
  planner.SetOptimizer(OptimizerType::NAIVE);
  PlannerResult result3 = planner.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  algorithms["DIJKTRA'S + NAIVE OPTIMIZER\t"] = result3.cost;

  // Improve on Dijkstra's via the "NLOPT" optimizer.
  planner.SetOptimizer(OptimizerType::NLOPT);
  PlannerResult result4 = planner.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  algorithms["DIJKTRA'S + NLOPT\t\t"] = result4.cost;

  // Now sorted by cost.
  std::multimap<double, std::string, std::greater<double>> algs_by_cost = 
    flip_map(algorithms);
    
  // Output the results
  std::cout << "\nPlanning Algorithm\t\tCost [hrs]" << std::endl;
  for ( auto iter = algs_by_cost.cbegin(); iter != algs_by_cost.cend(); ++iter) {
    std::cout << iter->second << std::setprecision(6) << iter->first
      << std::endl;
  }
  std::cout << "\n";

}
