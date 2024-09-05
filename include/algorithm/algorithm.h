#pragma once
/**
 * @file algorithm.h
 * @author Carl Stahoviak
 * @brief The path planning algorithm interface class.
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
   * @brief Stores the resulting output of a path planning algorithm. The result
   * contains both a path and a total cost (duration in hours) of the planned
   * route.
   */
  struct PlannerResult
  {
    std::vector<Node> route;
    double cost{0};
    double max_range{0};
    double speed{0};
  };

  /**
   * @brief Defines the public interface for all planning algorithm classes.
   */
  class PlanningAlgorithm
  {
    public:
      enum class AlgorithmType {
        ASTAR,
        DIJKSTRAS,
        NAIVE
      };

      enum class CostFunctionType {
        // So far, these only apply to the Naive planning algorithm.
        MINIMIZE_DIST_TO_NEXT,
        MINIMIZE_DIST_REMAINING,
        MINIMIZE_TIME_REMAINING,
        NONE
      };

      PlanningAlgorithm(RoutePlanner*);

      static std::unique_ptr<PlanningAlgorithm> GetPlanningAlgorithm(
        RoutePlanner*, AlgorithmType&&, CostFunctionType&&);
      
      virtual PlannerResult PlanRoute(const std::string&, const std::string&) = 0;
      virtual double ComputeCost(const Node* const, const Node* const) const = 0;

    protected:
      // Constructs the final route.
      virtual std::vector<Node> ConstructFinalRoute_(const Node* const) = 0;

      // Store a reference to the top-level RoutePlanner instance.
      RoutePlanner* route_planner_{nullptr};

      // Store all of the nodes in the network.
      std::unordered_map<std::string, Node> nodes_;

      // Utility functions
      double ComputeChargeTime_(const Node* const, const Node* const) const;
      double ComputeArrivalRange_(const Node* const, const Node* const) const;
      double ComputeDepartureRange_(const Node* const) const;
  };
} // end namespace supercharger
