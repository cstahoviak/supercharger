#pragma once
/**
 * @file dijkstras.h
 * @author Carl Stahoviak
 * @brief Dijkstra's path planning algorithm.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"

#include <functional>

namespace supercharger::algorithm
{
  /**
   * @brief Implements Dijkstra's algorithm.
   */
  class Dijkstras : public Planner
  {
    public:
      using DijkstrasCostFcnType =
        std::function<double(const Node&, const Node&, double)>;

      Dijkstras(DijkstrasCostFcnType cost_f) : cost_f(std::move(cost_f)) {};

      PlannerResult PlanRoute(
        const std::string&,
        const std::string&,
        double,
        double) override;

      double ComputeCost(const Node&, const Node&, double) const override;

    protected:
      std::vector<Node> ConstructFinalRoute_(const Node&) override;

    private:
      std::vector<std::shared_ptr<Node>> GetNeighbors_(const Node&, double);
      
      DijkstrasCostFcnType cost_f;
  };

  /**
   * @brief Implements a "simple" cost function for Dijkstra's algorithm.
   * 
   * @return double 
   */
  double SimpleCost(const Node&, const Node&, double);

  /**
   * @brief Implements an "optimized" cost function for Dijkstra's algorithm.
   * The optimized cost function uses a constrained optimization scheme to...
   * 
   * @return double 
   */
  double OptimizedCost(const Node&, const Node&, double);
} // end namespace supercharger::algorithm