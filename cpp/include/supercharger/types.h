#pragma once
/**
 * @file types.h
 * @author Carl Stahoviak
 * @brief Custom types for the Supercharger project.
 * @version 0.1
 * @date 2024-11-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/node.h"


namespace supercharger
{
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

    // Move assignment operator.
    PlannerResult& operator=(PlannerResult&&);

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
}