/**
 * @file test_planner.cpp
 * @author Carl Stahoviak
 * @brief Unit tests for the Planner class.
 * @version 0.1
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"
#include "supercharger/algorithm/dijkstras.h"

#include <gtest/gtest.h>

using namespace supercharger;
using namespace supercharger::algorithm;


TEST(TestPlanner, TestGetInvalidPlanner)
{
  // Ensure that attempting to create a "Naive" planner with a custom cost
  // function fails.
  EXPECT_THROW(
    Planner::GetPlanner(Planner::AlgorithmType::NAIVE, SimpleCost),
    std::invalid_argument
  );
}

TEST(TestPlanner, TestCopyPlannerResult)
{
  // Create the planner
  std::unique_ptr<Planner> planner =
    Planner::GetPlanner(Planner::CostFunctionType::DIJKSTRAS_SIMPLE);

  // Plan the route
  PlannerResult result = planner->Plan(
    "Council_Bluffs_IA", "Cadillac_MI", 320, 105);

  // Verify that each node (managed by a shared_ptr) has a use_count of 1.
  for ( const std::shared_ptr<Node>& node_ptr : result.route ) {
    EXPECT_EQ(node_ptr.use_count(), 1);
  }

  // Iterating by copy (rather than reference) should result in a use_count of 2.
  for ( const std::shared_ptr<Node> node_ptr : result.route ) {
    EXPECT_EQ(node_ptr.use_count(), 2);
  }

  // Copy the PlannerResult (uses the PlannerResult's copy ctor).
  PlannerResult copy(result);

  // Verify that top-level data was copied correctly.
  EXPECT_EQ(copy.cost, result.cost);
  EXPECT_EQ(copy.max_range, result.max_range);
  EXPECT_EQ(copy.speed, result.speed);

  // The copied PlannerResult should contain new nodes with identical info.
  EXPECT_EQ(copy.route.size(), result.route.size());
  for ( size_t idx = 0; idx < result.route.size(); idx++ ) {
    // Expect inequality of pointers.
    EXPECT_NE(copy.route[idx], result.route[idx]);

    // Expect equality of all other route data.
    EXPECT_EQ(copy.route[idx]->name(), result.route[idx]->name());
    EXPECT_EQ(copy.route[idx]->duration, result.route[idx]->duration);
    EXPECT_EQ(copy.route[idx]->arrival_range, result.route[idx]->arrival_range);
    EXPECT_EQ(copy.route[idx]->departure_range, result.route[idx]->departure_range);
  }

  // Verify that the use count for each node is still 1.
  for ( size_t idx = 0; idx < result.route.size(); idx++ ) {
    EXPECT_EQ(result.route[idx].use_count(), 1);
    EXPECT_EQ(copy.route[idx].use_count(), 1);
  }
}