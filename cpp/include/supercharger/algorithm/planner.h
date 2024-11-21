#pragma once
/**
 * @file algorithm.h
 * @author Carl Stahoviak
 * @brief The path planning algorithm interface class.
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/math.h"
#include "supercharger/node.h"

#include <memory>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>


namespace supercharger::algorithm
{
  using DijkstrasCostFcnType = 
    std::function<double(const DijkstrasNode&, const DijkstrasNode&, double, double)>;

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
    std::vector<std::shared_ptr<Node>> route;
    double cost{0};
    double max_range{0};
    double speed{0};

    PlannerResult() = default;
    PlannerResult(std::vector<std::shared_ptr<Node>>, double, double, double);

    // Copy constructor and copy assignment operator.
    PlannerResult(const PlannerResult&);
    PlannerResult& operator=(const PlannerResult&);

    const std::vector<double>& durations() {
      if ( durations_.size() != route.size() ) {
        durations_.clear();
        for ( const std::shared_ptr<const Node>& node : route ) {
          durations_.push_back(node->duration);
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
        NAIVE_MINIMIZE_DIST_TO_NEXT,
        NAIVE_MINIMIZE_DIST_REMAINING,
        NAIVE_MINIMIZE_TIME_REMAINING,
        DIJKSTRAS_SIMPLE,
        DIJKSTRAS_OPTIMIZED
      };

      // Factory functions for the Planner type.
      static std::unique_ptr<Planner> GetPlanner(CostFunctionType);
      static std::unique_ptr<Planner> GetPlanner(
        AlgorithmType, DijkstrasCostFcnType);
      
      /**
       * @brief Plans a route (acts as the public interface of the Planner).
       * 
       * @param origin The origin node.
       * @param destination The destination node.
       * @param max_range [km] The vehicle's max range.
       * @param speed [km/hr] The vehicle's constant velocity.
       * @return PlannerResult The planner result.
       */
      PlannerResult Plan(
        const std::string&,
        const std::string&,
        double,
        double);

    protected:
      /**
       * @brief Each Planner must implement its own PlanRoute_ function.
       * 
       * @return PlannerResult 
       */
      virtual PlannerResult PlanRoute_(
        const std::string&,
        const std::string&,
        double,
        double) = 0;

      /**
       * @brief The planner's cost function.
       * 
       * @param current The current node.
       * @param candidate The candidate "next" node.
       * @param max_range [km] The vehicle's max range.
       * @param speed [km/hr] The vehicle's constant velocity.
       * @return double The cost to reach the candidate node from the current
       * node.
       */
      virtual double ComputeCost_(
        const Node&, const Node&, double, double) const = 0;

      /**
       * @brief Constructs the final route.
       * 
       * @return std::vector<std::shared_ptr<Node>> 
       */
      virtual std::vector<std::shared_ptr<Node>> ConstructRoute_(const Node&) = 0;

      /**
       * @brief Initializes the node graph by resetting all nodes in the graph.
       */
      void InitializeNodeGraph_();

      // Store all of the nodes in the network.
      std::unordered_map<std::string, std::shared_ptr<Node>> nodes_;
  };

  // Overload the string stream operator for the AlgorithmType and CostFunctionType
  std::ostream& operator<<(std::ostream&, const Planner::AlgorithmType&);
  std::ostream& operator<<(std::ostream&, const Planner::CostFunctionType&);

} // end namespace supercharger::algorithm

