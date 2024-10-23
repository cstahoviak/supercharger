/**
 * @file test_algorithm.cpp
 * @author Carl Stahoviak
 * @brief Unit tests for the Planner and Planner-Derived classes.
 * @version 0.1
 * @date 2024-10-20
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/dijkstras.h"

#include <gtest/gtest.h>

using namespace supercharger::algorithm;

class DijkstrasPlannerTestFixture : public testing::Test
{
  protected:
    void SetUp() override
    {
      // Suppress all terminal output
      auto old_buffer = std::cout.rdbuf(nullptr);
    }

    // Create the route planner
    Dijkstras planner{SimpleCost};

    // Define the origin and destination charger names
    const std::string initial_charger_name{"Council_Bluffs_IA"};
    const std::string goal_charger_name{"Cadillac_MI"};

    // Define some vehicle constants
    const double max_range{320};
    const double speed{105};

    // The expected stops along the route planned by Dijkstra's.
    std::vector<std::string> stops {
      "Council_Bluffs_IA",
      "Worthington_MN",
      "Albert_Lea_MN",
      "Onalaska_WI",
      "Mauston_WI",
      "Sheboygan_WI",
      "Cadillac_MI"
    };
};

TEST_F(DijkstrasPlannerTestFixture, TestPlanRoute)
{
  // Plan the route
  PlannerResult result = planner.PlanRoute(
    initial_charger_name, goal_charger_name, max_range, speed);

  // Verify that each stop along the route is correct
  EXPECT_EQ(stops.size(), result.route.size());
  for( size_t idx = 0; idx < stops.size(); idx++ ) {
    EXPECT_EQ(result.route[idx].name(), stops[idx]);
  }

  // Verify that the total route cost is correct.
  EXPECT_DOUBLE_EQ(result.cost, 17.254757750681119);

  // Verify that the max range and speed are set correctly.
  EXPECT_DOUBLE_EQ(result.max_range, max_range);
  EXPECT_DOUBLE_EQ(result.speed, speed);
}

TEST_F(DijkstrasPlannerTestFixture, TestPlanMultipleRoutes)
{
  std::vector<PlannerResult> results;

  // Plan the route
  results.emplace_back(planner.PlanRoute(
    initial_charger_name, goal_charger_name, max_range, speed));

  // Try to plan the same route again
  results.emplace_back(planner.PlanRoute(
    initial_charger_name, goal_charger_name, max_range, speed));

  for ( const PlannerResult& result : results ) {
    // Verify that each stop along the route is correct
    EXPECT_EQ(stops.size(), result.route.size());
    for( size_t idx = 0; idx < stops.size(); idx++ ) {
      EXPECT_EQ(result.route[idx].name(), stops[idx]);
    }

    // Verify that the total route cost is correct.
    EXPECT_DOUBLE_EQ(result.cost, 17.254757750681119);

    // Verify that the max range and speed are set correctly.
    EXPECT_DOUBLE_EQ(result.max_range, max_range);
    EXPECT_DOUBLE_EQ(result.speed, speed);
  }
}

TEST_F(DijkstrasPlannerTestFixture, TestInsufficientRange)
{
  // TODO: Verify that the planner fails cleanly if an insifficient vehicle
  // range is provided.
}