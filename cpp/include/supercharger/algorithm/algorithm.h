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
#include "supercharger/math/math.h"
#include "supercharger/node.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace supercharger
{
  // Forward declare the RoutePlanner
  // NOTE: MUST include "planner.h" in "algorithm.cpp"
  class RoutePlanner;

  namespace algorithm
  {
    /**
     * @brief Stores the resulting output of a path planning algorithm. The
     * result contains both a path and a total cost (duration in hours) of the
     * planned route.
     */
    struct PlannerResult
    {
      std::vector<Node> route;
      double cost{0};
      double max_range{0};
      double speed{0};

      PlannerResult() = default;
      PlannerResult(std::vector<Node>, double, double, double);

      const std::vector<double>& durations() {
        if ( durations_.empty() ) {
          for ( const Node& node : route ) {
            durations_.push_back(node.duration);
          }
        }
        return durations_;
      }

      private:
        std::vector<double> durations_;
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
          RoutePlanner*, AlgorithmType, CostFunctionType);
        
        virtual PlannerResult PlanRoute(const std::string&, const std::string&) = 0;
        virtual double ComputeCost(const Node&, const Node&) const = 0;
        
        void Reset();

      protected:
        // Constructs the final route.
        virtual std::vector<Node> ConstructFinalRoute_(const Node&) = 0;

        // Store a reference to the top-level RoutePlanner instance.
        RoutePlanner* route_planner_{nullptr};

        // Store all of the nodes in the network.
        std::unordered_map<std::string, std::shared_ptr<Node>> nodes_;

        // Utility functions
        double ComputeChargeTime_(const Node&, const Node&) const;
        double ComputeArrivalRange_(const Node&, const Node&) const;
        double ComputeDepartureRange_(const Node&) const;
    };
  } // end namespace supercharger::algorithm

} // end namespace supercharger

