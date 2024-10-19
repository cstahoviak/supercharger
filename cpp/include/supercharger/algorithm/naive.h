#pragma once
/**
 * @file brute_force.h
 * @author Carl Stahoviak
 * @brief My "naive" path planner.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"

namespace supercharger::algorithm
{
  /**
   * @brief My "naive" path planner.
   */
  class NaivePlanner : public Planner
  {
    public:
      NaivePlanner(CostFunctionType type) : type_(type) {};

      PlannerResult PlanRoute(
        const std::string&,
        const std::string&,
        double,
        double) override;
        
      double ComputeCost(const Node&, const Node&, double) const override;

    protected:
      std::vector<Node> ConstructFinalRoute_(const Node&) override;

    private:
      Charger destination_;
      CostFunctionType type_;

      // Store the planned route.
      std::vector<std::shared_ptr<Node>> route_;

      // Store the "naive" cost function weight parameters (weight remaining
      // travel time more heavily than charge time) NOTE: These weights were
      // arrived at after a bit of tuning, but more work could be done here to
      // further refine these values.
      const double weight_time_to_destination_ = 0.75;
      const double weight_time_to_charge_ = 0.25;

      // Store the total cost (time in hrs) of the route.
      double total_cost_{0};

      // The "naive" route planner is implemented as a recursive algorithm.
      void PlanRouteRecursive_(
        const std::string&,
        const std::string&,
        double,
        double);
      
      void UpdateRouteCost_(double);
  };
} // end namespace supercharger