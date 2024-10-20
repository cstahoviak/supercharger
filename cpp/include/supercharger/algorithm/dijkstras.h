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
      Dijkstras() = default;
      Dijkstras(
        std::function<double(const Node&, const Node&, double, void*)> cost_f) :
          cost_f(std::move(cost_f)) {};


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
      
      std::function<double(const Node&, const Node&, double, void*)> cost_f;
      void* cost_data_;
  };
} // end namespace supercharger::algorithm