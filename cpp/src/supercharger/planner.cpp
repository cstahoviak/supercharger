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
#include "supercharger/algorithm/algorithm.h"
#include "supercharger/logging.h"
#include "supercharger/math/math.h"
#include "supercharger/planner.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>

namespace supercharger
{
  /**
   * @brief Formats the final route output to comply with the format expected
   * by the provided "checker" executables.
   * 
   * @param stream 
   * @param route 
   * @return std::ostream& 
   */
  std::ostream& operator<<(std::ostream& stream, const std::vector<Node>& route)
  {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const Node& node : route ) {
      stream << node;
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  std::ostream& operator<<(
    std::ostream& stream, const std::vector<std::shared_ptr<Node>>& route)
  {
    size_t sz = route.size();
    size_t idx = 0;
    for ( const std::shared_ptr<Node>& node : route ) {
      stream << *node;
      if ( idx < sz - 1 ) {
        stream << ", ";
      }
      idx++;
    }
    return stream;
  }

  RoutePlanner::RoutePlanner(
    AlgoType algo_type,
    CostFcnType cost_type,
    OptimizerType optimizer_type) {
    // Create the network map (must do this before creating the planning algo).
    for ( const Charger& charger : supercharger::network ) {
      // const std::pair<std::unordered_map<std::string, Charger>::iterator, bool> pair =
      const auto& pair = network_.try_emplace(charger.name, &charger);
      if ( !pair.second ) {
        DEBUG("Charger '" << charger.name << "' already exists in the network. "
          << "Skipping.");
      }
    }

    // Create the planning algorithm.
    // NOTE: Even though this function accepts 'algo_type' and 'cost_type as 
    // r-value references, once we're within the scope of this function, they
    // become l-values (pretty odd, right?). So we need to use std::move() to 
    // cast the l-values 'algo_type' and 'cost_type' as r-values to "move" them
    // to PlanningAlgorithm::GetPlanner.
    planning_algo_ = PlanningAlgorithm::GetPlanningAlgorithm(
      this, algo_type, cost_type);

    // Create the optimizer.
    optimizer_ = Optimizer::GetOptimizer(optimizer_type);

  }

  PlannerResult RoutePlanner::PlanRoute(
    const std::string& origin, const std::string& destination)
  {
    // Initialize the route.
    Initialize_(origin, destination);

    // Plan the route.
    DEBUG("Planning route between '" << origin << "' and '" << destination 
      << "'.");
    PlannerResult result = planning_algo_.get()->PlanRoute(origin, destination);

    if ( result.route.back().name() != destination_.name ) {
      INFO("Search terminated. Solution not found.");
      return {};
    }

    DEBUG("Solution found!");
    return ( optimizer_ ) ? optimizer_.get()->Optimize(result) : result;    
  }

  void RoutePlanner::SetPlanningAlgorithm(
    AlgoType algo_type, CostFcnType cost_type)
  {
    std::unique_ptr<PlanningAlgorithm> new_algo = 
      PlanningAlgorithm::GetPlanningAlgorithm(
        this, std::move(algo_type), std::move(cost_type));
    planning_algo_.swap(new_algo);
  }

  void RoutePlanner::SetOptimizer(OptimizerType type) {
    // Reset the planning algorithm.
    planning_algo_.get()->Reset();

    // Swap the optimizer.
    std::unique_ptr<Optimizer> new_optimizer = 
      Optimizer::GetOptimizer(type);
    optimizer_.swap(new_optimizer);
  }

  void RoutePlanner::Initialize_(
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