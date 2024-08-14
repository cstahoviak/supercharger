/**
 * @file planner.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "logging.h"
#include "math.h"
#include "planner.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

namespace supercharger
{
  using AlgoType = PlanningAlgorithm::AlgorithmType;
  using CostFcnType = PlanningAlgorithm::CostFunctionType;

  std::ostream& operator<<(std::ostream& stream, const std::vector<Stop>& route) {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const Stop& stop : route ) {
      stream << stop.charger->name;
      if ( stop.duration > 0 ) {
        stream << ", " << std::setprecision(5) << stop.duration;
      }
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  RoutePlanner::RoutePlanner(AlgoType algo_type, CostFcnType cost_type = CostFcnType::NONE)
  {
    // Create the planning algorithm
    planning_algo_ = PlanningAlgorithm::GetPlanner(algo_type, cost_type);
    planning_algo_->SetPlanner(this);

    // Create the network map
    for ( Charger& charger : supercharger::network ) {
      // const std::pair<std::unordered_map<std::string, Charger>::iterator, bool> pair =
      const auto& pair = network_.try_emplace(charger.name, &charger);
      if ( !pair.second ) {
        LOG("Charger '" << charger.name << "' already exists in the network. "
          << "Skipping.");
      }
    }
  }

  std::vector<Stop> RoutePlanner::PlanRoute(const std::string& origin, const std::string& destination) {
    // Initialize the route
    std::vector<Stop> route = InitializeRoute_(origin, destination);

    // Plan the route
    LOG("Planning route between '" << origin << "' and '" << destination << "'");
    planning_algo_.get()->PlanRoute(route);

    if ( route.back().charger->name == destination_->name ) {
      LOG("\nSolution found!");
    }
    else {
      LOG("\nSearch terminated. Solution not found.");
    }

    // TODO: Backtrace to determine the route (doesn't apply to current
    // solution method).
    
    return route;
  }

  std::vector<Stop> RoutePlanner::InitializeRoute_(const std::string& origin, const std::string& destination) {
    // Store the origin and destination chargers
    try {
      origin_ = network_.at(origin); 
    }
    catch( const std::out_of_range& e) {
      std::ostringstream os;
      os << "Initial charger '" << origin << "' not in network.";
      throw std::out_of_range(os.str());
    };

    try {
      destination_ = network_.at(destination); 
    }
    catch( const std::out_of_range& e) {
      std::ostringstream os;
      os << "Destination charger '" << destination << "' not in network.";
      throw std::out_of_range(os.str());
    };

    // Initialize the route and add the origin
    std::vector<Stop> route;
    route.emplace_back(origin_, 0, max_range_, nullptr);
    return route;
  }
} // end namespace supercharger