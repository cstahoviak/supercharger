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
  // TODO: These aliases probably shouldn't be in the top-level namespace.
  using AlgoType = algorithm::Planner::AlgorithmType;
  using CostFcnType = algorithm::Planner::CostFunctionType;
  using DijkstrasCostFcnType = algorithm::DijkstrasCostFcnType;
  using OptimizerType = optimize::Optimizer::OptimizerType;

  using PlannerResult = algorithm::PlannerResult;
  using Planner = algorithm::Planner;
  using Optimizer = optimize::Optimizer;

  class Supercharger
  {    
    public:
      Supercharger(CostFcnType, OptimizerType);
      Supercharger(AlgoType, DijkstrasCostFcnType, OptimizerType);

      // The following ctors are known as "delegating" ctors.
      Supercharger(CostFcnType cost_type) :
        Supercharger(cost_type, OptimizerType::NONE) {};

      Supercharger(AlgoType algo_type, DijkstrasCostFcnType cost_f) :
        Supercharger(algo_type, cost_f, OptimizerType::NONE) {};

      /**
       * @brief Plan the route with the provided Planner and Optimizer.
       * 
       * @param origin The origin node.
       * @param destination The destination node.
       * @return PlannerResult The planner result.
       */
      PlannerResult PlanRoute(const std::string&, const std::string&);

      // TODO: Maybe for simplicity, there should should only be two of these
      // Set...() functions: one that accepts a Planner, and one that accepts an
      // Optimizer.
      void SetPlanner(CostFcnType);
      void SetPlanner(AlgoType, DijkstrasCostFcnType);

      void SetOptimizer(OptimizerType);

      // Getters
      const std::unordered_map<std::string, const Charger*>& network() const
      { 
        return network_;
      }
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

      /**
       * @brief Initializes the network map.
       */
      void InitializeNetwork_();

      /**
       * @brief Sets the origin and destination chargers and validates the
       * "max_range" and "speed" values. 
       * 
       * @param origin The origin node.
       * @param destination The destinatiion node.
       */
      void InitializeRoute_(const std::string&, const std::string&);
  };
} // end namespace supercharger
