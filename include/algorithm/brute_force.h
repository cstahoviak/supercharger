#pragma once
/**
 * @file brute_force.h
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
   * @brief My "brute force" path planner.
   */
  class BruteForce : public PlanningAlgorithm
  {
    public:
      // NOTE: Call the base class via initializer list "constructor chaining"
      BruteForce(RoutePlanner* rp, CostFunctionType type) : 
        PlanningAlgorithm(rp), type_(type) {};

      void PlanRoute(std::vector<Stop>&) override;

    private:
      CostFunctionType type_;

      // Store "brute force" const function weight parameters (weight remaining
      // travel time more heavily than charge time) NOTE: These weights were
      // arrived at after a bit of tuning, but more work could be done here to
      // further refine these values.
      const double weight_time_to_destination_ = 0.75;
      const double weight_time_to_charge_ = 0.25;

      void UpdateRouteCost_(const std::vector<Stop>&);

      // The "brute force" algorithm cost function
      double ComputeCost_(const Charger* const, const Charger* const) const;
  };
} // end namespace supercharger