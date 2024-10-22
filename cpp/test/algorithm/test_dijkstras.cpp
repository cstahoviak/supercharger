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

class PlannerTestFixture : public testing::Test
{
  protected:
    void SetUp() override
    {
      // Suppress all terminal output
      auto old_buffer = std::cout.rdbuf(nullptr);
    }

    const std::string initial_charger_name{"Council_Bluffs_IA"};
    const std::string goal_charger_name{"Cadillac_MI"};

    const double max_range{320};
    const double speed{105};
};

TEST_F(PlannerTestFixture, TestDijkstrasPlanner)
{
  // Create the route planner
  auto planner = Dijkstras(SimpleCost);

  // Plan the route
  PlannerResult result = planner.PlanRoute(
    initial_charger_name, goal_charger_name, max_range, speed);

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

  // Verify that the number of nodes is correct.
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