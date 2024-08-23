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

      static std::unique_ptr<PlanningAlgorithm> GetPlanner(
        AlgorithmType&&, CostFunctionType&&);
      
      virtual void PlanRoute(std::vector<Stop>&) = 0;

      double ComputeDistance(const Charger* const, const Charger* const) const;

      // RoutePlanner setter
      void SetRoutePlanner(RoutePlanner* planner) { route_planner_ = planner; }

      // Getters
      const double cost() const { return total_cost_; }

    protected:
      // Store a reference to top-level RoutePlanner instance
      RoutePlanner* route_planner_{nullptr};

      // Store the total cost (time in hrs) of the route
      double total_cost_{0};
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
