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

namespace supercharger::algorithm
{
  /**
   * @brief Stores the resulting output of a path planning algorithm. The
   * result contains both a path and a total cost (duration in hours) of the
   * planned route.
   * 
   * TODO: Think of a way to distinguish the output of a Planner
   * from the output of an Optimizer.
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
      if ( durations_.size() != route.size() ) {
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
  class Planner
  {
    public:
      enum class AlgorithmType {
        ASTAR,
        DIJKSTRAS,
        NAIVE
      };

      enum class CostFunctionType {
        // So far, these only apply to the Naive planning algorithm.
        // TODO: This enum should probably fall within the scope of the
        // NaivePlanner class.
        MINIMIZE_DIST_TO_NEXT,
        MINIMIZE_DIST_REMAINING,
        MINIMIZE_TIME_REMAINING,
        NONE
      };

      Planner();

      static std::unique_ptr<Planner> GetPlanner(
        AlgorithmType, CostFunctionType);
      
      virtual PlannerResult PlanRoute(
        const std::string&,
        const std::string&,
        double,
        double) = 0;

      virtual double ComputeCost(const Node&, const Node&, double) const = 0;
      
      void Reset();

    protected:
      // Constructs the final route.
      virtual std::vector<Node> ConstructFinalRoute_(const Node&) = 0;

      // Store all of the nodes in the network.
      std::unordered_map<std::string, std::shared_ptr<Node>> nodes_;

      // Utility functions
      // TODO: In order to implement the Dijkstra's cost function as a
      // callable via std::function, it might be necessary to decouple these
      // functions from the Planner class. Is that a good idea?
      // Free function vs. member function?
      // Note: If no private/protected data from class is used, make it a free
      // function.
      double ComputeChargeTime_(const Node&, const Node&) const;
      double ComputeArrivalRange_(const Node&, const Node&) const;
      double ComputeDepartureRange_(const Node&) const;
  };
} // end namespace supercharger

