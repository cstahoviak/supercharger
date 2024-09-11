#pragma once
/**
 * @file planner.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/algorithm.h"
#include "optimizer/optimizer.h"
#include "network.h"
#include "node.h"

#include <memory>
#include <ostream>
#include <unordered_map>
#include <vector>


namespace supercharger
{  
  using AlgoType = PlanningAlgorithm::AlgorithmType;
  using CostFcnType = PlanningAlgorithm::CostFunctionType;

  class RoutePlanner
  {
    public:
      // NOTE: Default parameter values must appear in the declaration.
      // NOTE: Initially, both ctor args were rvalue references, but a more
      // common pattern is to consume by value and std::move in the initializer
      // list.
      RoutePlanner(AlgoType, CostFcnType = CostFcnType::NONE);

      // TODO: Add ctor that also takes max range and speed.

      PlannerResult PlanRoute(const std::string&, const std::string&);
      PlannerResult OptimizeRoute(const PlannerResult&) const;

      // Getters
      const std::unordered_map<std::string, const Charger*>& network() const
      { 
        return network_;
      }
      const Charger& destination() const { return destination_; }
      const double max_range() const { return max_range_; }
      const double speed() const { return speed_; }

      // Setters
      double& max_range() { return max_range_; }
      double& speed() { return speed_; }

    private:
      Charger origin_;
      Charger destination_;

      // Store some hard-coded constants.
      double max_range_{0};   // [km]
      double speed_{0};       // [km/hr]

      // Store the network of chargers.
      std::unordered_map<std::string, const Charger*> network_;

      // Store the path planning algorithm.
      std::unique_ptr<PlanningAlgorithm> planning_algo_;

      // Store the route optimizer.
      std::unique_ptr<Optimizer> optimizer_;

      void Initialize_(const std::string&, const std::string&);
  };

  // Overload the string stream operator to output the route
  std::ostream& operator<<(std::ostream&, const std::vector<Node>&);
  std::ostream& operator<<(std::ostream&, const std::vector<std::shared_ptr<Node>>&);

} // end namespace supercharger
