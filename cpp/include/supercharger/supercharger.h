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
#include "supercharger/utils.h"

#include <memory>
#include <ostream>
#include <unordered_map>
#include <vector>


namespace supercharger
{  
  using AlgoType = algorithm::Planner::AlgorithmType;
  using NaiveCostFcnType = algorithm::Planner::NaiveCostType;
  using DijkstrasCostFcnType = algorithm::DijkstrasCostFcnType;
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
      Supercharger(
        AlgoType,
        NaiveCostFcnType,
        DijkstrasCostFcnType,
        OptimizerType
      );
      // Supercharger(
      //   AlgoType,
      //   NaiveCostFcnType = NaiveCostFcnType::NAIVE,
      //   DijkstrasCostFcnType = nullfunc,
      //   OptimizerType = OptimizerType::NONE
      // );

      // The following ctors are known as "delegating" ctors.
      Supercharger(NaiveCostFcnType naive_cost_type) :
        Supercharger(AlgoType::NAIVE, naive_cost_type, {}, OptimizerType::NONE) {};

      Supercharger(NaiveCostFcnType naive_cost_type, OptimizerType optim_type) :
        Supercharger(AlgoType::NAIVE, naive_cost_type, {}, optim_type) {};

      Supercharger(DijkstrasCostFcnType cost_f) :
        Supercharger(AlgoType::DIJKSTRAS, NaiveCostFcnType::NONE, cost_f, OptimizerType::NONE) {};

      Supercharger(DijkstrasCostFcnType cost_f, OptimizerType optim_type) :
        Supercharger(AlgoType::DIJKSTRAS, NaiveCostFcnType::NONE, cost_f, optim_type) {};

      PlannerResult PlanRoute(const std::string&, const std::string&);

      // TODO: Maybe for simplicity, there should should only be two of these
      // Set...() functions: one that accepts a Planner, and one that accepts an
      // Optimizer.
      void SetPlanner(NaiveCostFcnType naive_cost_type) {
        SetPlanner_(AlgoType::NAIVE, naive_cost_type, nullcostfunc);
      }
      void SetPlanner(DijkstrasCostFcnType cost_f) {
        SetPlanner_(AlgoType::DIJKSTRAS, NaiveCostFcnType::NONE, cost_f);
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
      void SetPlanner_(AlgoType, NaiveCostFcnType, DijkstrasCostFcnType);

  };
} // end namespace supercharger
