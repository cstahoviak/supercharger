/**
 * @file algorithm.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
// #include "algorithm/algorithm.h"
#include "algorithm/brute_force.h"
#include "logging.h"
#include "math.h"


namespace supercharger
{
  // NOTE: Do NOT repeat the 'static keyword at the cpp file level
  // NOTE: I think this must be defined at the cpp file level because otherwise
  // I get a "not declared in this scope error" related to the derived planning
  // algorithm types (BruteForce, Dijkstra's, etc.). 
  std::unique_ptr<PlanningAlgorithm> PlanningAlgorithm::GetPlanner(
    AlgorithmType&& algo_type,
    CostFunctionType&& cost_type = CostFunctionType::NONE)
  {
    switch ( algo_type )
    {
      case AlgorithmType::BRUTE_FORCE:
        // NOTE: There is an important difference between public and protected
        // inheritance when it comes to the compiler "being aware" that a
        // unique_ptr of the base class (the return type of this function) can
        // be initialized from a unique_ptr of the derived class: public
        // inheritance allows this, but protected inheritance does not. But why?
        return std::make_unique<BruteForce>(cost_type);

      case AlgorithmType::DIJKSTRAS:
        // return std::make_unique<Dijstras>();
        return std::unique_ptr<PlanningAlgorithm>(nullptr);

      case AlgorithmType::ASTAR:
        return std::unique_ptr<PlanningAlgorithm>(nullptr);
        // return std::make_unique<AStar>();
      
      default:
        return std::unique_ptr<PlanningAlgorithm>(nullptr);
    }
  }

  /**
   * @brief Effectively a wrapper around the great_circle_distance() function.
   * 
   * @param charger1
   * @param charger2
   * @return double 
   */
  double PlanningAlgorithm::ComputeDistance(
    const Charger* const charger1, const Charger* const charger2) const
  {
    return great_circle_distance(
      charger1->lat, charger1->lon, charger2->lat, charger2->lon);
  }

  void AStar::PlanRoute(std::vector<Stop>& route) {
    return;
  }

} // end namespace supercharger