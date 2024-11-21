/**
 * @file test_astar.cpp
 * @author Carl Stahoviak
 * @brief Unit tests for the A* algorithm (which isn't implemented yet).
 * @version 0.1
 * @date 2024-10-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "supercharger/algorithm/astar.h"

#include <gtest/gtest.h>

using namespace supercharger::algorithm;


class AStarPlannerTestFixture : public testing::Test
{
  protected:
    void SetUp() override
    {
      // Suppress all terminal output
      auto old_buffer = std::cout.rdbuf(nullptr);
    }

    // Create the route planner
    AStar planner{SimpleCost};

    // Define the origin and destination charger names
    const std::string initial_charger_name{"Council_Bluffs_IA"};
    const std::string goal_charger_name{"Cadillac_MI"};

    // Define some vehicle constants
    const double max_range{320};
    const double speed{105};
};

TEST_F(AStarPlannerTestFixture, TestPlanRoute)
{
  // Verify that a runtime exception is thrown
  EXPECT_THROW(
    planner.Plan(initial_charger_name, goal_charger_name, max_range, speed),
    std::runtime_error
  );
}