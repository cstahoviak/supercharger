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
      RoutePlanner(AlgoType&&, CostFcnType&& = CostFcnType::NONE);

      // TODO: Add ctor that also takes max range and speed.

      PlannerResult PlanRoute(const std::string&, const std::string&);
      std::vector<Node> OptimizeRoute(const std::vector<Node>&) const;

      // Getters
      const std::unordered_map<std::string, Charger*>& network() const { return network_; }
      const Charger* const destination() const { return destination_; }
      const double max_range() const { return max_range_; }
      const double speed() const { return speed_; }

      // Setters
      double& max_range() { return max_range_; }
      double& speed() { return speed_; }

      // Return the total cost (time in hours) of the planned route.
      const double cost() const { return planning_algo_->cost(); }

    private:
      // TODO: Switch to unique_ptr?
      Charger* origin_{nullptr};
      Charger* destination_{nullptr};

      // Store some hard-coded constants.
      double max_range_{0};   // [km]
      double speed_{0};       // [km/hr]

      // Store the network of chargers.
      std::unordered_map<std::string, Charger*> network_;

      // Store the path planning algorithm.
      std::unique_ptr<PlanningAlgorithm> planning_algo_;

      void Initialize_(const std::string&, const std::string&);
  };

  // Overload the string stream operator to output the route
  std::ostream& operator<<(std::ostream&, const std::vector<Node>&);
  std::ostream& operator<<(std::ostream&, const std::vector<Node*>&);

} // end namespace supercharger
