#pragma once
/**
 * @file algorithm.h
 * @author Carl Stahoviak
 * @brief The path planning algorithms.
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
// #include "planner.h"
#include "stop.h"

#include <memory>
#include <string>
#include <vector>

namespace supercharger
{
  // Forward declare the RoutePlanner
  // NOTE: MUST include "planner.h" in "algorithm.cpp"
  class RoutePlanner;

  /**
   * @brief Defines the public interface for all planning algorithm classes.
   */
  class PlanningAlgorithm
  {
    public:
      enum class AlgorithmType {
        BRUTE_FORCE,
        DIJKSTRAS,
        ASTAR
      };

      enum class CostFunctionType {
        // So far, these only apply to the Brute Force algorithm
        MINIMIZE_DIST_REMAINING,
        MINIMIZE_TIME_REMAINING,
        NONE
      };

      static std::unique_ptr<PlanningAlgorithm> GetPlanner(AlgorithmType&&, CostFunctionType&&);
      virtual void PlanRoute(std::vector<Stop>&) = 0;

      double ComputeDistance(const Charger* const, const Charger* const) const;

      // RoutePlanner setter and getter
      void SetRoutePlanner(RoutePlanner* planner) { route_planner_ = planner; }
      // const RoutePlanner* const route_planner() const { return route_planner_; }

      const double cost() const { return total_cost_; }

    protected:
      // Store a reference to top-level RoutePlanner instance
      RoutePlanner* route_planner_{nullptr};

      // Store the total cost of the route
      double total_cost_{0};
  };

  /**
   * @brief My "brute force" path planner.
   */
  class BruteForce : public PlanningAlgorithm
  {
    public:
      BruteForce(CostFunctionType type) : type_(type) {};

      void PlanRoute(std::vector<Stop>&) override;

    private:
      CostFunctionType type_;

      // Store "brute force" const function weight parameters (weight remaining
      // travel time more heavily than charge time) NOTE: These weights were
      // arrived at after a bit of tuning, but more work could be done here to
      // further refine these values.
      const double weight_time_to_destination_ = 0.75;
      const double weight_time_to_charge_ = 0.25;

      // Utility functions (args are const pointers to const Chargers)
      double ComputeChargeTime_(const Stop&, const Charger* const) const;
      void UpdateCost_(const std::vector<Stop>&);

      // The "brute force" algorithm cost function
      double ComputeCost_(const Charger* const, const Charger* const) const;
  };

  /**
   * @brief Implements Dijkstra's algorithm.
   */
  class Dijkstras : public PlanningAlgorithm
  {
    public:
      void PlanRoute(std::vector<Stop>&) override;
  };

  /**
   * @brief Implements the A* search algorithm. A* makes sense in this scenario
   * because the total cost of the route is computed as a function of both the
   * distance (total time traveled) from the current node to the start node as 
   * well as a heuristic that, in our case, should be a function of the charge
   * rate.
   */
  class AStar : public PlanningAlgorithm
  {
    public:
      void PlanRoute(std::vector<Stop>&) override;
  };
  
} // end namespace supercharger
