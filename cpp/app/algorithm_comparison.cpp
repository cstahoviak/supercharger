/**
 * @file algorithm_comparison.cpp
 * @author Carl Stahoviak
 * @brief A comparison of the planning and optimization algorithm currently
 * implemented.
 * @version 0.1
 * @date 2024-10-01
 * 
 * TODO: Valgrind provides the following summary of memory leaks. Only the
 * "Naive" planner has a leak.
 * 
 * ==37798== LEAK SUMMARY:
 * ==37798==    definitely lost: 128 bytes in 1 blocks
 * ==37798==    indirectly lost: 738 bytes in 6 blocks
 * ==37798==      possibly lost: 0 bytes in 0 blocks
 * ==37798==    still reachable: 0 bytes in 0 blocks
 * ==37798==         suppressed: 0 bytes in 0 blocks
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/logging.h"
#include "supercharger/supercharger.h"

// #include <glog/logging.h>

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

  // TODO: Add glog
  // google::InitGoogleLogging(argv[0]);
  
  // Parse the initial and goal charger names
  const std::string initial_charger_name = argv[1];
  const std::string goal_charger_name = argv[2];

  // Store a map of planning algorithms and route costs
  std::map<std::string, double> algorithms;

  // Create the Route Planner
  Supercharger app(NaiveCostFcnType::MINIMIZE_TIME_REMAINING);

  // Set the vehicle's speed and max range
  app.max_range() = 320;
  app.speed() = 105;

  // Test to make sure the network getter works.
  const auto& network = app.network();

  // Plan the route via the "Naive" route planner
  PlannerResult result1 = app.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  algorithms["NAIVE\t\t\t\t"] = result1.cost;

  // Plan the route via Dijkstra's algorithm.
  app.SetPlanner(AlgoType::DIJKSTRAS, algorithm::SimpleCost);
  PlannerResult result2 = app.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  algorithms["DIJKTRA'S\t\t\t"] = result2.cost;

  // Improve on Dijkstra's via the "naive" optimizer.
  app.SetOptimizer(OptimizerType::NAIVE);
  PlannerResult result3 = app.PlanRoute(
    initial_charger_name,
    goal_charger_name
  );
  algorithms["DIJKTRA'S + NAIVE OPTIMIZER\t"] = result3.cost;

  // Improve on Dijkstra's via the "NLOPT" optimizer.
  app.SetOptimizer(OptimizerType::NLOPT);
  PlannerResult result4 = app.PlanRoute(
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
