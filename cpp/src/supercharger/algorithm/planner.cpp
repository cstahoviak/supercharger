/**
 * @file algorithm.cpp
 * @author Carl Stahoviak
 * @brief Defines the Planner interface functions.
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/algorithm/planner.h"
#include "supercharger/algorithm/astar.h"
#include "supercharger/algorithm/naive.h"
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/logging.h"
#include "supercharger/supercharger.h"

#include <algorithm>
#include <sstream>


namespace supercharger::algorithm
{
  PlannerResult::PlannerResult(
    std::vector<Node> route, double cost, double max_range, double speed) :
    route(std::move(route)), cost(cost), max_range(max_range), speed(speed) {}

  Planner::Planner() {
    // Create a set of nodes from the charger network.
    for ( const Charger& charger : supercharger::network ) {
      // TODO: I should be able to use try_emplace to construct the shared_ptr
      // in place rather than moving it, but I haven't gotten it to work.
      std::shared_ptr<Node> node = std::make_shared<Node>(charger);
      nodes_.try_emplace(charger.name, std::move(node));
    }
  }

  // NOTE: I think this must be defined at the cpp file level because otherwise
  // I get a "not declared in this scope error" related to the derived planner
  // types (Naive, Dijkstra's, etc.). 
  std::unique_ptr<Planner> Planner::GetPlanner(
    CostFunctionType cost_type)
  {
    static const std::unordered_map<CostFunctionType, AlgorithmType> 
      cost_type_to_algo_type
    {
      {CostFunctionType::NAIVE_MINIMIZE_DIST_TO_NEXT, AlgorithmType::NAIVE},
      {CostFunctionType::NAIVE_MINIMIZE_DIST_REMAINING, AlgorithmType::NAIVE},
      {CostFunctionType::NAIVE_MINIMIZE_TIME_REMAINING, AlgorithmType::NAIVE},
      {CostFunctionType::DIJKSTRAS_SIMPLE, AlgorithmType::DIJKSTRAS},
      {CostFunctionType::DIJKSTRAS_OPTIMIZED, AlgorithmType::DIJKSTRAS},
    };

    AlgorithmType algo_type = cost_type_to_algo_type.at(cost_type);
    switch ( algo_type )
    {
      case AlgorithmType::NAIVE:
        // NOTE: There is an important difference between public and protected
        // inheritance when it comes to the compiler "being aware" that a
        // unique_ptr of the base class (the return type of this function) can
        // be initialized from a unique_ptr of the derived class: public
        // inheritance allows this, but protected inheritance does not. But why?
        return std::make_unique<NaivePlanner>(cost_type);

      case AlgorithmType::DIJKSTRAS:
        return std::make_unique<Dijkstras>(cost_type);

      case AlgorithmType::ASTAR:
        return std::make_unique<AStar>(cost_type);
      
      default:
        std::ostringstream os;
        os << "The cost function type '" << cost_type << "' is not " <<
          "associated with any Planner types.";
        throw std::invalid_argument(os.str());
    }
  }

  std::unique_ptr<Planner> Planner::GetPlanner(
    AlgorithmType algo_type,
    DijkstrasCostFcnType cost_fcn)
  {
    switch ( algo_type )
    {
      case AlgorithmType::DIJKSTRAS:
        return std::make_unique<Dijkstras>(cost_fcn);

      case AlgorithmType::ASTAR:
        return std::make_unique<AStar>(cost_fcn);
      
      default:
        std::ostringstream os;
        os << "The '" << algo_type << "' Planner (AlgorithmType: " << 
        algo_type << ") cannot be created with a custom cost function.";
        throw std::invalid_argument(os.str());
    }
  }

  void Planner::Reset() {
    for ( auto [name, node] : nodes_ ) {
      node.get()->Reset();
    }
  }

  void Planner::InitializeNodeGraph_() {
    for (const auto& [name, node] : nodes_ ) {
      if ( node.get()->visited ) {
        // If even a single node is marked 'visited', the graph must be reset.
        Reset();
        break;
      }
    }
  }

  std::ostream& operator<<(std::ostream& stream, const Planner::AlgorithmType& type)
  {
    using AlgoType = Planner::AlgorithmType;

    static const std::unordered_map<AlgoType, std::string> map {
      {AlgoType::ASTAR, "ASTAR"},
      {AlgoType::DIJKSTRAS, "DIJKSTRAS"},
      {AlgoType::NAIVE, "NAIVE"},
    };

    return stream << map.at(type);
  }

  std::ostream& operator<<(std::ostream& stream, const Planner::CostFunctionType& type)
  {
    using CostFcnType = Planner::CostFunctionType;

    static const std::unordered_map<CostFcnType, std::string> map {
      {CostFcnType::NAIVE_MINIMIZE_DIST_TO_NEXT, "NAIVE_MINIMIZE_DIST_TO_NEXT"},
      {CostFcnType::NAIVE_MINIMIZE_DIST_REMAINING, "NAIVE_MINIMIZE_DIST_REMAINING"},
      {CostFcnType::NAIVE_MINIMIZE_TIME_REMAINING, "NAIVE_MINIMIZE_TIME_REMAINING"},
    };

    return stream << map.at(type);
  }

  double GetChargeTime(const Node& current, const Node& next)
  {
    // Compute the distance to the next charger.
    double current_to_next = math::distance(current, next);

    // Compute the charge time required to make it to the next charger.
    // NOTE: we're charging the car only enough to make it to the next node.
    double charge_time = 
      (current_to_next - current.arrival_range) / current.charger().rate;

    // If the charge time is negative, we have sufficient range without
    // additional charging to reach the next node.
    return std::max(0.0, charge_time);
  }

  double GetArrivalRange(const Node& current, const Node& next)
  {
    // The range remaining after arriving at the next node is the departure
    // range at the current node - the distance to the next charger.
    return current.departure_range - math::distance(current, next);
  }

  double GetDepartureRange(const Node& current)
  {
    // The departure range at the current node is the arrival range at the
    // current node + range added by charging.
    return current.arrival_range + current.duration * current.charger().rate;
  }
} // end namespace supercharger::algorithm