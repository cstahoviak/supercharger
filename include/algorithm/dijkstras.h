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

      void PlanRoute(std::vector<Node>&) override;
      double ComputeCost(const Node&, const Charger* const) const override;

    private:
      std::vector<Node*> GetNeighbors_(const Node* const);
      void ConstructFinalRoute_(const Node* const, std::vector<Node>&);
  };
} // end namespace supercharger