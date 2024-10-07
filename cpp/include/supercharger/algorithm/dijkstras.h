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
#include "supercharger/algorithm/algorithm.h"

namespace supercharger::algorithm
{
  /**
   * @brief Implements Dijkstra's algorithm.
   */
  class Dijkstras : public PlanningAlgorithm
  {
    public:
      Dijkstras(RoutePlanner* rp) : PlanningAlgorithm(rp) {};

      PlannerResult PlanRoute(const std::string&, const std::string&) override;
      double ComputeCost(const Node&, const Node&) const override;

    protected:
      std::vector<Node> ConstructFinalRoute_(const Node&) override;

    private:
      std::vector<std::shared_ptr<Node>> GetNeighbors_(const Node&);
      
  };
} // end namespace supercharger