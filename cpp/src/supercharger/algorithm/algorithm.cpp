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
#include "supercharger/algorithm/algorithm.h"
#include "supercharger/algorithm/astar.h"
#include "supercharger/algorithm/naive.h"
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/logging.h"
#include "supercharger/planner.h"

#include <algorithm>


namespace supercharger::algorithm
{
  PlannerResult::PlannerResult(
    std::vector<Node> route, double cost, double max_range, double speed) :
    route(std::move(route)), cost(cost), max_range(max_range), speed(speed) {}

  /**
   * @brief PlanningAlgorithm constructor.
   * 
   * @param rp 
   */
  PlanningAlgorithm::PlanningAlgorithm(RoutePlanner* rp) : route_planner_(rp) {
    // Create a set of nodes from the route planner's charger network.
    for (const auto& [name, charger] : route_planner_->network() ) {
      // TODO: I should be able to use try_emplace to construct the shared_ptr
      // in place rather than moving it, but I haven't gotten it to work.
      std::shared_ptr<Node> node = std::make_shared<Node>(*charger);
      nodes_.try_emplace(name, std::move(node));
    }
  }

  // NOTE: Do NOT repeat the 'static' keyword at the cpp file level.
  // NOTE: I think this must be defined at the cpp file level because otherwise
  // I get a "not declared in this scope error" related to the derived planning
  // algorithm types (Naive, Dijkstra's, etc.). 
  std::unique_ptr<PlanningAlgorithm> PlanningAlgorithm::GetPlanningAlgorithm(
    RoutePlanner* rp,
    AlgorithmType algo_type,
    CostFunctionType cost_type)
  {
    switch ( algo_type )
    {
      case AlgorithmType::NAIVE:
        // NOTE: There is an important difference between public and protected
        // inheritance when it comes to the compiler "being aware" that a
        // unique_ptr of the base class (the return type of this function) can
        // be initialized from a unique_ptr of the derived class: public
        // inheritance allows this, but protected inheritance does not. But why?
        return std::make_unique<NaivePlanner>(rp, cost_type);

      case AlgorithmType::DIJKSTRAS:
        return std::make_unique<Dijkstras>(rp);

      case AlgorithmType::ASTAR:
        return std::make_unique<AStar>(rp);
      
      default:
        return std::unique_ptr<PlanningAlgorithm>(nullptr);
    }
  }

  /**
   * @brief Resets all nodes in the graph.
   */
  void PlanningAlgorithm::Reset() {
    for ( auto [name, node] : nodes_ ) {
      node.get()->Reset();
    }
  }

  /**
   * @brief Computes the charge time at the current node required to make it to
   * the next node.
   * 
   * @param current The current Node.
   * @param next The next node.
   * @return double The charge time required to make it to the next node. The
   * arrival range at the 'next' node will be zero.
   */
  double PlanningAlgorithm::ComputeChargeTime_(
    const Node& current, const Node& next) const
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

  /**
   * @brief Computes the arrival range at the 'next' node after departing the
   * 'current' node.
   * 
   * @param current The current node.
   * @param next The next node.
   * @return double The arrival range at the 'next' node.
   */
  double PlanningAlgorithm::ComputeArrivalRange_(
    const Node& current, const Node& next) const
  {
    // The range remaining after arriving at the next node is the departure
    // range at the current node - the distance to the next charger.
    return current.departure_range - math::distance(current, next);
  }

  /**
   * @brief Computes the departure range at the current node given the current
   * node's arrival range and charging duration.
   * 
   * @param current The current node.
   * @return double The departure range after charging at the current node.
   */
  double PlanningAlgorithm::ComputeDepartureRange_(const Node& current) const {
    // The departure range at the current node is the arrival range at the
    // current node + range added by charging.
    return current.arrival_range + current.duration * current.charger().rate;
  }
} // end namespace supercharger::algorithm