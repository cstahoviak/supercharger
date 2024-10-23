/**
 * @file supercharger.cpp
 * @author Carl Stahoviak
 * @brief The Supercharger class definition.
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"
#include "supercharger/logging.h"
#include "supercharger/math/math.h"
#include "supercharger/supercharger.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>


namespace supercharger
{
  Supercharger::Supercharger(
    AlgoType algo_type,
    NaiveCostFcnType naive_cost_type,
    DijkstrasCostFcnType cost_f,
    OptimizerType optimizer_type)
  {
    // Create the network map (must do this before creating the planner).
    for ( const Charger& charger : supercharger::network ) {
      // const std::pair<std::unordered_map<std::string, Charger>::iterator, bool> pair =
      const auto& pair = network_.try_emplace(charger.name, &charger);
      if ( !pair.second ) {
        DEBUG("Charger '" << charger.name << "' already exists in the network. "
          << "Skipping.");
      }
    }

    // Create the planner.
    planner_ = Planner::GetPlanner(algo_type, naive_cost_type, cost_f);

    // Create the optimizer.
    optimizer_ = Optimizer::GetOptimizer(optimizer_type);
  }

  PlannerResult Supercharger::PlanRoute(
    const std::string& origin, const std::string& destination)
  {
    // Initialize the route.
    Initialize_(origin, destination);

    // Plan the route.
    DEBUG("Planning route between '" << origin << "' and '" << destination 
      << "'.");
    PlannerResult result = 
      planner_.get()->PlanRoute(origin, destination, max_range_, speed_);

    if ( result.route.back().name() != destination_.name ) {
      INFO("Search terminated. Solution not found.");
      return {};
    }

    DEBUG("Solution found!");
    return ( optimizer_ ) ? optimizer_.get()->Optimize(result) : result;    
  }

  void Supercharger::SetOptimizer(OptimizerType type) {
    // Swap the optimizer.
    std::unique_ptr<Optimizer> new_optimizer = 
      Optimizer::GetOptimizer(type);
    optimizer_.swap(new_optimizer);
  }

  void Supercharger::SetPlanner_(
    AlgoType algo_type, NaiveCostFcnType cost_type, DijkstrasCostFcnType cost_f)
  {
    // Swap the planner.
    std::unique_ptr<Planner> new_algo = 
      Planner::GetPlanner(algo_type, cost_type, cost_f);
    planner_.swap(new_algo);
  }

  void Supercharger::Initialize_(
    const std::string& origin, const std::string& destination)
  {
    // Store the origin and destination chargers.
    try { 
      origin_ = *network_.at(origin); 
    }
    catch( const std::out_of_range& e ) {
      std::ostringstream os;
      os << "Initial charger '" << origin << "' not in network.";
      throw std::out_of_range(os.str());
    };

    try {
      destination_ = *network_.at(destination); 
    }
    catch( const std::out_of_range& e ) {
      std::ostringstream os;
      os << "Destination charger '" << destination << "' not in network.";
      throw std::out_of_range(os.str());
    };

    // TODO: max_range_ should actually be greater than or equal to the
    // starting node's nearest neighbor.
    if ( max_range_ <= 0 ) {
      std::ostringstream os;
      os << "The vehicle's maximum range value of " << max_range_ << "km " <<
        "is invalid. The vehicle's max range must be greater than or equal " <<
        "to the nearest neighbor of the starting node.";
      throw std::runtime_error(os.str());
    }

    if ( speed_ <= 0 ) {
      std::ostringstream os;
      os << "The vehicle's speed value of " << speed_ << " km/hr is " <<
      "invalid. The vehicle's speed must be greater than zero.";
      throw std::runtime_error(os.str());
    }
  }
} // end namespace supercharger