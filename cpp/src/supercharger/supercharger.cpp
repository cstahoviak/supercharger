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

  Supercharger::Supercharger(
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
    planner_ = Planner::GetPlanner(algo_type, cost_type);

    // Create the optimizer.
    optimizer_ = Optimizer::GetOptimizer(optimizer_type);
  }

  /**
   * @brief Plan the route with the provided Planner and Optimizer.
   * 
   * @param origin The origin node.
   * @param destination The destination node.
   * @return PlannerResult The planner result.
   */
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
      // TODO: "failure_modes" segfaults before it gets here.
      INFO("Search terminated. Solution not found.");
      return {};
    }

    DEBUG("Solution found!");
    return ( optimizer_ ) ? optimizer_.get()->Optimize(result) : result;    
  }

  void Supercharger::SetPlanningAlgorithm(
    AlgoType algo_type, CostFcnType cost_type)
  {
    std::unique_ptr<Planner> new_algo = 
      Planner::GetPlanner(algo_type, cost_type);
    planner_.swap(new_algo);
  }

  void Supercharger::SetOptimizer(OptimizerType type) {
    // Reset the planning algorithm.
    planner_.get()->Reset();

    // Swap the optimizer.
    std::unique_ptr<Optimizer> new_optimizer = 
      Optimizer::GetOptimizer(type);
    optimizer_.swap(new_optimizer);
  }

  /**
   * @brief Sets the origin and destination chargers and validates the
   * "max_range" and "speed" values. 
   * 
   * @param origin The origin node.
   * @param destination The destinatiion node.
   */
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