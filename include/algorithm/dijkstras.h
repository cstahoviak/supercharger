#pragma once
/**
 * @file dijkstras.h
 * @author Carl Stahoviak
 * @brief Dijkstra's path planning algorithm.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/algorithm.h"

namespace supercharger
{
  /**
   * @brief Implements Dijkstra's algorithm.
   */
  class Dijkstras : public PlanningAlgorithm
  {
    public:
      Dijkstras(RoutePlanner* rp) : PlanningAlgorithm(rp) {};

      PlannerResult PlanRoute(const std::string&, const std::string&) override;
      double ComputeCost(const Node* const, const Node* const) const override;

    protected:
      std::vector<Node> ConstructFinalRoute_(const Node* const) override;

    private:
      std::vector<Node*> GetNeighbors_(const Node* const);
      
  };
} // end namespace supercharger