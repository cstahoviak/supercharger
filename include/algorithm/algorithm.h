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
#include "math.h"
#include "node.h"

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

      PlanningAlgorithm(RoutePlanner*);

      static std::unique_ptr<PlanningAlgorithm> GetPlanningAlgorithm(
        RoutePlanner*, AlgorithmType&&, CostFunctionType&&);
      
      virtual void PlanRoute(std::vector<Node>&) = 0;
      virtual double ComputeCost(const Node&, const Charger* const) const = 0;

      // Getters
      const double cost() const { return total_cost_; }

    protected:
      // Store a reference to top-level RoutePlanner instance
      RoutePlanner* route_planner_{nullptr};

      // Store all of the nodes (Nodes) in the network
      std::unordered_map<std::string, Node> nodes_;

      // Store the total cost (time in hrs) of the route
      double total_cost_{0};

      // Utility functions
      double ComputeChargeTime_(const Node&, const Charger* const) const;
      double ComputeRangeRemaining_(const Node&, const Charger* const) const;
  };

  /**
   * @brief Effectively a wrapper around the great_circle_distance() function.
   * 
   * TODO: Would it make more sense for this to be in math.h?
   * 
   * @param charger1
   * @param charger2
   * @return double 
   */
  inline double ComputeDistance(
    const Charger* const charger1, const Charger* const charger2)
  {
    return great_circle_distance(
      charger1->lat, charger1->lon, charger2->lat, charger2->lon);
  }
  
} // end namespace supercharger
