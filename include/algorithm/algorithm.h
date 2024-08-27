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
#include <unordered_map>
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
        MINIMIZE_DIST_TO_NEXT,
        MINIMIZE_DIST_REMAINING,
        MINIMIZE_TIME_REMAINING,
        NONE
      };

      PlanningAlgorithm(RoutePlanner* planner);

      static std::unique_ptr<PlanningAlgorithm> GetPlanningAlgorithm(
        RoutePlanner*, AlgorithmType&&, CostFunctionType&&);
      
      virtual void PlanRoute(std::vector<Stop>&) = 0;

      // Getters
      const double cost() const { return total_cost_; }

    protected:
      // Store a reference to top-level RoutePlanner instance
      RoutePlanner* route_planner_{nullptr};

      // Store all of the nodes (Stops) in the network
      std::unordered_map<std::string, Stop> nodes_;

      // Store the total cost (time in hrs) of the route
      double total_cost_{0};

      // Utility functions (args are const pointers to const Chargers)
      double ComputeDistance_(const Charger* const, const Charger* const) const;
      double ComputeChargeTime_(const Stop&, const Charger* const) const;
      double ComputeRangeRemaining_(const Stop&, const Charger* const) const;
  };
  
} // end namespace supercharger
