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

     enum class CostFunctionType {
        NAIVE_MINIMIZE_DIST_TO_NEXT,
        NAIVE_MINIMIZE_DIST_REMAINING,
        NAIVE_MINIMIZE_TIME_REMAINING,
        DIJKSTRAS_SIMPLE,
        DIJKSTRAS_OPTIMIZED
      };

      /**
       * @brief Planner constructor.
       * 
       * TODO: It might make more sense (or be more explicit) for the Charger
       * network to be passed in as an input arg.
       */
      Planner();

      static std::unique_ptr<Planner> GetPlanner(CostFunctionType);

      static std::unique_ptr<Planner> GetPlanner(
        AlgorithmType, DijkstrasCostFcnType);
      
      virtual PlannerResult PlanRoute(
        const std::string&,
        const std::string&,
        double,
        double) = 0;

      virtual double ComputeCost(const Node&, const Node&, double) const = 0;
      
      /**
       * @brief Resets all nodes in the graph.
       */
      void Reset();

    protected:
      /**
       * @brief Initializes the node graph.
       * 
       * TODO: Is there a good way to implement something like this at the
       * base-class level rather than requiring each derived class to call an
       * "initialization" function?
       */
      void InitializeNodeGraph_();

      /**
       * @brief Constructs the final route.
       * 
       * @return std::vector<Node> 
       */
      virtual std::vector<Node> ConstructFinalRoute_(const Node&) = 0;

      // Store all of the nodes in the network.
      std::unordered_map<std::string, std::shared_ptr<Node>> nodes_;
  };

  // Overload the string stream operator for the AlgorithmType and CostFunctionType
  std::ostream& operator<<(std::ostream&, const Planner::AlgorithmType&);
  std::ostream& operator<<(std::ostream&, const Planner::CostFunctionType&);

  /**
   * @brief Computes the charge time at the current node required to make it to
   * the next node.
   * 
   * @param current The current Node.
   * @param next The next node.
   * @return double The charge time required to make it to the next node.
   * Assumes that the arrival range at the 'next' node will be zero.
   */
  double GetChargeTime(const Node&, const Node&);

  /**
   * @brief Computes the arrival range at the 'next' node after departing the
   * 'current' node.
   * 
   * @param current The current node.
   * @param next The next node.
   * @return double The arrival range at the 'next' node.
   */
  double GetArrivalRange(const Node&, const Node&);

  /**
   * @brief Computes the departure range at the current node given the current
   * node's arrival range and charging duration.
   * 
   * @param current The current node.
   * @return double The departure range after charging at the current node.
   */
  double GetDepartureRange(const Node&);

} // end namespace supercharger::algorithm

