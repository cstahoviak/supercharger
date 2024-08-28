/**
 * @file algorithm.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/algorithm.h"
#include "algorithm/astar.h"
#include "algorithm/naive.h"
#include "algorithm/dijkstras.h"
#include "logging.h"
#include "planner.h"


namespace supercharger
{
  PlanningAlgorithm::PlanningAlgorithm(RoutePlanner* rp) : route_planner_(rp) {
    // Create a set of nodes (nodes) from the route planner's charger network
    for (const auto& [name, charger] : route_planner_->network() ) {
      nodes_.try_emplace(name, charger);
    }
  }

  // NOTE: Do NOT repeat the 'static keyword at the cpp file level
  // NOTE: I think this must be defined at the cpp file level because otherwise
  // I get a "not declared in this scope error" related to the derived planning
  // algorithm types (BruteForce, Dijkstra's, etc.). 
  std::unique_ptr<PlanningAlgorithm> PlanningAlgorithm::GetPlanningAlgorithm(
    RoutePlanner* rp,
    AlgorithmType&& algo_type,
    CostFunctionType&& cost_type)
  {
    switch ( algo_type )
    {
      case AlgorithmType::NAIVE:
        // NOTE: There is an important difference between public and protected
        // inheritance when it comes to the compiler "being aware" that a
        // unique_ptr of the base class (the return type of this function) can
        // be initialized from a unique_ptr of the derived class: public
        // inheritance allows this, but protected inheritance does not. But why?
        return std::make_unique<Naive>(rp, cost_type);

      case AlgorithmType::DIJKSTRAS:
        return std::make_unique<Dijkstras>(rp);

      case AlgorithmType::ASTAR:
        return std::make_unique<AStar>(rp);
      
      default:
        return std::unique_ptr<PlanningAlgorithm>(nullptr);
    }
  }

  double PlanningAlgorithm::ComputeChargeTime_(
    const Node& current_node, const Charger* const next_charger) const
  {
    // Compute the distance to the next charger
    double current_to_next = 
      ComputeDistance(current_node.charger, next_charger);

    // Compute the charge time required to make it to the next charger.
    // NOTE: we're charging the car only enough to make it to the next node.
    return (current_to_next - current_node.range) / current_node.charger->rate;
  }

  double PlanningAlgorithm::ComputeRangeRemaining_(
    const Node& current_node, const Charger* const next) const
  {
    // The range remaining is the range at the current node + the range added by
    // charging at the current node - the distance to the next charger.
    return current_node.range + 
      (current_node.duration * current_node.charger->rate) -
      ComputeDistance(current_node.charger, next);
  }
} // end namespace supercharger