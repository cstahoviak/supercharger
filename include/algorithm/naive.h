#pragma once
/**
 * @file brute_force.h
 * @author Carl Stahoviak
 * @brief My "naive" path planner.
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
   * @brief My "naive" path planner.
   */
  class Naive : public PlanningAlgorithm
  {
    public:
      // NOTE: Calls the base class via initializer list "constructor chaining"
      Naive(RoutePlanner* rp, CostFunctionType type) : 
        PlanningAlgorithm(rp), type_(type) {};

      void PlanRoute(std::vector<Node>&) override;
      double ComputeCost(const Node&, const Charger* const) const override;

    private:
      CostFunctionType type_;

      // Store the "naive" cost function weight parameters (weight remaining
      // travel time more heavily than charge time) NOTE: These weights were
      // arrived at after a bit of tuning, but more work could be done here to
      // further refine these values.
      const double weight_time_to_destination_ = 0.75;
      const double weight_time_to_charge_ = 0.25;

      void UpdateRouteCost_(const std::vector<Node>&);

      // The "naive" algorithm cost function
      double ComputeCost_(const Charger* const, const Charger* const) const;
  };
} // end namespace supercharger