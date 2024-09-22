/**
 * @file test_nloptimizer.cpp
 * @author your name (you@domain.com)
 * @brief Unit tests for math functions
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "optimizer/nloptimizer.h"
#include "planner.h"

#include <gtest/gtest.h>

using namespace supercharger;
using ConstraintData = NLOptimizer::ConstraintData;

class NLOptimizerTestFixture : public testing::Test
{
  protected:
    void SetUp() override
    {
      // Define the initial and goal charger names
      const std::string initial_charger_name = "Council_Bluffs_IA";
      const std::string goal_charger_name = "Cadillac_MI";

      // Create the Route Planner
      RoutePlanner planner(AlgoType::DIJKSTRAS, OptimizerType::NLOPT);

      // Set the vehicle's speed and max range
      planner.max_range() = 320;
      planner.speed() = 105;

      // Plan the route
      result = planner.PlanRoute(
        initial_charger_name,
        goal_charger_name
      );

      // Define the problem dimensionality and create the constraint data
      n = result.route.size() - 2;
      constr_data = NLOptimizer::CreateConstraintData(result);
    }

    PlannerResult result;
    ConstraintData constr_data;
    unsigned n;
};

TEST_F(NLOptimizerTestFixture, TestGetArrivalRanges) {
  // Define the charging duration for all nodes not including the first and
  // last nodes
  std::vector<double> durations;
  for (auto iter = result.route.cbegin() + 1; iter != result.route.cend() - 1; ++iter)
  {
    durations.push_back(iter->duration);
  }

  const std::vector<double> arrival_ranges = 
    get_arrival_ranges(durations, &constr_data);

  // For Dijkstra's algorithm, we expect the arrival ranges to be zero for all
  // nodes from node 3 onward.
  for (size_t idx = 0; idx < arrival_ranges.size(); idx++) {
    EXPECT_DOUBLE_EQ(
      arrival_ranges.at(idx),result.route.at(idx + 2).arrival_range);
  }
}