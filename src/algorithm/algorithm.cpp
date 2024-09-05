/**
 * @file algorithm.cpp
 * @author Carl Stahoviak
 * @brief Defines the PlanningAlgorithm interface functions.
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

#include <algorithm>


namespace supercharger
{
  PlanningAlgorithm::PlanningAlgorithm(RoutePlanner* rp) : route_planner_(rp) {
    // Create a set of nodes from the route planner's charger network.
    for (const auto& [name, charger] : route_planner_->network() ) {
      nodes_.try_emplace(name, charger);
    }
  }

  // NOTE: Do NOT repeat the 'static' keyword at the cpp file level.
  // NOTE: I think this must be defined at the cpp file level because otherwise
  // I get a "not declared in this scope error" related to the derived planning
  // algorithm types (Naive, Dijkstra's, etc.). 
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

  /**
   * @brief Computes the charge time at the current node required to make it to
   * the next node.
   * 
   * @param current 
   * @param next 
   * @return double 
   */
  double PlanningAlgorithm::ComputeChargeTime_(
    const Node* const current, const Node* const next) const
  {
    // Compute the distance to the next charger.
    double current_to_next = compute_distance(current, next);

    // Compute the charge time required to make it to the next charger.
    // NOTE: we're charging the car only enough to make it to the next node.
    double charge_time = 
      (current_to_next - current->arrival_range) / current->charger->rate;

    // If the charge time is negative, we have sufficient range without
    // additional charging to reach the next node.
    return std::max(0.0, charge_time);
  }

  double PlanningAlgorithm::ComputeArrivalRange_(
    const Node* const current, const Node* const next) const
  {
    // The range remaining after arriving at the next node is the departure
    // range at the current node - the distance to the next charger.
    return current->departure_range - compute_distance(current, next);
  }

  double PlanningAlgorithm::ComputeDepartureRange_(
    const Node* const current) const
  {
    // The departure range at the current node is the arrival range at the
    // current node + range added by charging.
    return current->arrival_range + current->duration * current->charger->rate;
  }
} // end namespace supercharger