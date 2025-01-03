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
  PlannerResult Planner::Plan(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    // Initialize the node graph and plan the route.
    InitializeNodeGraph_();
    return PlanRoute_(origin, destination, max_range, speed);
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

  void Planner::InitializeNodeGraph_() {
    for ( auto [name, node] : nodes_ ) {
      node.get()->Reset();
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
} // end namespace supercharger::algorithm