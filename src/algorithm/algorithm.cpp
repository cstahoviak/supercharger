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
#include "algorithm/brute_force.h"
#include "algorithm/dijkstras.h"
#include "logging.h"
#include "math.h"
#include "planner.h"


namespace supercharger
{
  PlanningAlgorithm::PlanningAlgorithm(RoutePlanner* rp) : route_planner_(rp) {
    // Create a set of nodes (stops) from the route planner's charger network
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
      case AlgorithmType::BRUTE_FORCE:
        // NOTE: There is an important difference between public and protected
        // inheritance when it comes to the compiler "being aware" that a
        // unique_ptr of the base class (the return type of this function) can
        // be initialized from a unique_ptr of the derived class: public
        // inheritance allows this, but protected inheritance does not. But why?
        return std::make_unique<BruteForce>(rp, cost_type);

      case AlgorithmType::DIJKSTRAS:
        return std::make_unique<Dijkstras>(rp);

      case AlgorithmType::ASTAR:
        return std::make_unique<AStar>(rp);
      
      default:
        return std::unique_ptr<PlanningAlgorithm>(nullptr);
    }
  }

  /**
   * @brief Effectively a wrapper around the great_circle_distance() function.
   * 
   * @param charger1
   * @param charger2
   * @return double 
   */
  double PlanningAlgorithm::ComputeDistance_(
    const Charger* const charger1, const Charger* const charger2) const
  {
    return great_circle_distance(
      charger1->lat, charger1->lon, charger2->lat, charger2->lon);
  }

  double PlanningAlgorithm::ComputeChargeTime_(
    const Stop& current_stop, const Charger* const next_charger) const
  {
    // Compute the distance to the next charger
    double current_to_next = 
      ComputeDistance_(current_stop.charger, next_charger);

    // Compute the charge time required to make it to the next charger.
    // NOTE: we're charging the car only enough to make it to the next stop.
    return (current_to_next - current_stop.range) / current_stop.charger->rate;
  }

  double PlanningAlgorithm::ComputeRangeRemaining_(
    const Stop& current_stop, const Charger* const next) const
  {
    // The range remaining is the range at the current stop + the range added by
    // charging at the current stop - the distance to the next charger.
    return current_stop.range + 
      (current_stop.duration * current_stop.charger->rate) -
      ComputeDistance_(current_stop.charger, next);
  }

  void AStar::PlanRoute(std::vector<Stop>& route) {
    return;
  }

} // end namespace supercharger