#pragma once
/**
 * @file dijkstras.h
 * @author your name (you@domain.com)
 * @brief 
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
      // TODO: Do I need to define a default constructor for use with
      // std::make_unique?

      void PlanRoute(std::vector<Stop>&) override;

    private:
      std::vector<Stop*> GetNeighbors_(Stop* const);
      void ConstructFinalRoute_(Stop* const, std::vector<Stop>&);
  };
} // end namespace supercharger