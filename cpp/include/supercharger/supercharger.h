#pragma once
/**
 * @file planner.h
 * @author Carl Stahoviak
 * @brief 
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"
#include "supercharger/optimize/optimizer.h"
#include "supercharger/network.h"
#include "supercharger/node.h"

#include <memory>
#include <ostream>
#include <unordered_map>
#include <vector>


namespace supercharger
{  
  using AlgoType = algorithm::Planner::AlgorithmType;
  using CostFcnType = algorithm::Planner::CostFunctionType;
  using OptimizerType = optimize::Optimizer::OptimizerType;

  using PlannerResult = algorithm::PlannerResult;
  using Planner = algorithm::Planner;
  using Optimizer = optimize::Optimizer;

  class Supercharger
  {
    public:
      // NOTE: Initially, the ctor args were rvalue references, but a more
      // common pattern is to consume by value and std::move in the initializer
      // list.
      Supercharger(AlgoType, CostFcnType, OptimizerType);

      // The following ctors are known as "delegating" ctors.
      Supercharger(AlgoType algo_type, CostFcnType cost_type) : 
        Supercharger(algo_type, cost_type, OptimizerType::NONE) {};

      Supercharger(AlgoType algo_type, OptimizerType optim_type) : 
        Supercharger(algo_type, CostFcnType::NONE, optim_type) {};

      // TODO: Now I have to add a ctor that adds a std::function type to its
      // signature? Having so many different overloaded ctors feels kind of
      // bloated to me. Maybe there's a better way to do this? It would be nice
      // to decouple the PlanningAlgorithms from the Supercharger and maybe this
      // is a good opportunity to do that.

      Supercharger(AlgoType algo_type) : 
        Supercharger(algo_type, CostFcnType::NONE, OptimizerType::NONE) {};

      PlannerResult PlanRoute(const std::string&, const std::string&);

      void SetPlanningAlgorithm(AlgoType, CostFcnType);
      void SetPlanningAlgorithm(AlgoType algo_type) {
        SetPlanningAlgorithm(algo_type, CostFcnType::NONE);
      }
      void SetOptimizer(OptimizerType);

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

      // Store some vehicle-related constants.
      double max_range_{0};   // [km]
      double speed_{0};       // [km/hr]

      // Store the network of chargers.
      // TODO: Maybe just copy the Chargers because they're lightweight enough
      // and there are only 300 of them. That way we don't have a pointer to
      // something that could possibly be changed elsewhere.
      std::unordered_map<std::string, const Charger*> network_;

      // Store the path planning algorithm.
      std::unique_ptr<Planner> planner_;

      // Store the route optimizer.
      std::unique_ptr<Optimizer> optimizer_;

      void Initialize_(const std::string&, const std::string&);
  };

  // Overload the string stream operator to output the route
  std::ostream& operator<<(std::ostream&, const std::vector<Node>&);
  std::ostream& operator<<(std::ostream&, const std::vector<std::shared_ptr<Node>>&);

} // end namespace supercharger
