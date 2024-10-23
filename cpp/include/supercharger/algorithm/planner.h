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
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace supercharger::algorithm
{
  using DijkstrasCostFcnType = 
    std::function<double(const Node&, const Node&, double)>;

  /**
   * @brief Stores the resulting output of a path planning algorithm. The
   * result contains both a path and a total cost (duration in hours) of the
   * planned route.
   * 
   * TODO: Think of a way to distinguish the output of a Planner from the
   * output of an Optimizer.
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
        durations_.clear();
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

     enum class NaiveCostType {
        MINIMIZE_DIST_TO_NEXT,
        MINIMIZE_DIST_REMAINING,
        MINIMIZE_TIME_REMAINING,
        NONE
      };

      Planner();

      static std::unique_ptr<Planner> GetPlanner(
        AlgorithmType, NaiveCostType, DijkstrasCostFcnType);
      
      virtual PlannerResult PlanRoute(
        const std::string&,
        const std::string&,
        double,
        double) = 0;

      virtual double ComputeCost(const Node&, const Node&, double) const = 0;
      
      void Reset();

    protected:
      // Initializes the node graph.
      void InitializeNodeGraph_();

      // Constructs the final route.
      virtual std::vector<Node> ConstructFinalRoute_(const Node&) = 0;

      // Store all of the nodes in the network.
      std::unordered_map<std::string, std::shared_ptr<Node>> nodes_;
  };

  // Utility functions
  // RoT: If no private/protected data from class is used, make it a free fcn.
  double GetChargeTime(const Node&, const Node&);
  double GetArrivalRange(const Node&, const Node&);
  double GetDepartureRange(const Node&);

} // end namespace supercharger

