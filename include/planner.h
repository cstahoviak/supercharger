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
#include "algorithm.h"
#include "network.h"
#include "stop.h"

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
      // TODO: Change to accepting r-value reference?
      RoutePlanner(AlgoType&&, CostFcnType&&);

      // TODO: Add ctor that also takes max range and speed

      std::vector<Stop> PlanRoute(const std::string&, const std::string&);

      // Getters
      const std::unordered_map<std::string, Charger*>& network() const { return network_; }
      const Charger* const destination() const { return destination_; }
      const double max_range() const { return max_range_; }
      const double speed() const { return speed_; }

      // Return the total cost (time) of the planned route
      const double cost() const { return planning_algo_->cost(); }

    private:
      // TODO: Switch to unique_ptr?
      Charger* origin_;
      Charger* destination_;

      // Store some hard-coded constants
      const double max_range_{320};   // [km]
      const double speed_{105};       // [km/hr]

      // Store the network
      std::unordered_map<std::string, Charger*> network_;

      // Store the path planning algorithm
      std::unique_ptr<PlanningAlgorithm> planning_algo_;

      std::vector<Stop> InitializeRoute_(const std::string&, const std::string&);
  };

  // Overload the string stream operator to output the route
  std::ostream& operator<<(std::ostream&, const std::vector<Stop>&);

} // end namespace supercharger
