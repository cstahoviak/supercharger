/**
 * @file test_supercharger.cpp
 * @author Carl Stahoviak
 * @brief Unit tests for the Supercharger class.
 * @version 0.1
 * @date 2024-10-22
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/optimize/optimizer.h"
#include "supercharger/supercharger.h"

#include <gtest/gtest.h>

using namespace supercharger;


class SuperchargerTestFixture : public testing::Test
{
  protected: 
    void SetUp() override
    {
      // Suppress all terminal output
      auto old_buffer = std::cout.rdbuf(nullptr);
    }

    Supercharger supercharger {
      Planner::CostFunctionType::DIJKSTRAS_SIMPLE,
      OptimizerType::NLOPT
    };

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

TEST_F(SuperchargerTestFixture, TestSetMaxRange)
{
  // Set the vehicle's max range
  supercharger.max_range() = max_range;
  EXPECT_DOUBLE_EQ(supercharger.max_range(), max_range);
}

TEST_F(SuperchargerTestFixture, TestSetSpeed)
{
  // Set the vehicle's speed
  supercharger.speed() = speed;
  EXPECT_DOUBLE_EQ(supercharger.speed(), speed);
}

TEST_F(SuperchargerTestFixture, TestPlanRoute)
{
  // Set the vehicle's max_range and speed
  supercharger.max_range() = max_range;
  supercharger.speed() = speed;

  // Plan the route
  auto result = supercharger.PlanRoute(initial_charger_name, goal_charger_name);

  // Verify that each stop along the route is correct
  EXPECT_EQ(stops.size(), result.route.size());
  for( size_t idx = 0; idx < stops.size(); idx++ ) {
    EXPECT_EQ(result.route[idx].name(), stops[idx]);
  }

  // Verify that the total route cost is correct.
  EXPECT_DOUBLE_EQ(result.cost, 16.843755271092608);

  // Verify that the max range and speed are set correctly.
  EXPECT_DOUBLE_EQ(result.max_range, max_range);
  EXPECT_DOUBLE_EQ(result.speed, speed);
  
}

TEST_F(SuperchargerTestFixture, TestInvalidOriginCharger)
{
  // Set the vehicle's max_range and speed
  supercharger.max_range() = max_range;
  supercharger.speed() = speed;

  // Set an origin that does not correspond to a valid charger
  const std::string origin = "Nederland_CO";

  // Atempt to plan the route with an invalid origin.
  EXPECT_THROW(
    supercharger.PlanRoute(origin, goal_charger_name),
    std::out_of_range
  );
}

TEST_F(SuperchargerTestFixture, TestInvalidDestinationCharger)
{
  // Set the vehicle's max_range and speed
  supercharger.max_range() = max_range;
  supercharger.speed() = speed;

  // Set an origin that does not correspond to a valid charger
  const std::string destination = "Nederland_CO";

  // Atempt to plan the route with an invalid origin.
  EXPECT_THROW(
    supercharger.PlanRoute(initial_charger_name, destination),
    std::out_of_range
  );
}

TEST_F(SuperchargerTestFixture, TestInvalidMaxRange)
{
  // Atempt to plan the route without setting the vehicle's max range or speed
  EXPECT_THROW(
    supercharger.PlanRoute(initial_charger_name, goal_charger_name),
    std::runtime_error
  );
}

TEST_F(SuperchargerTestFixture, TestInvalidSpeed)
{
  // Set the vehicle's max range, but not the speed
  supercharger.max_range() = max_range;

  // Atempt to plan the route without setting the vehicle's max range or speed
  EXPECT_THROW(
    supercharger.PlanRoute(initial_charger_name, goal_charger_name),
    std::runtime_error
  );

  // Set an invalid speed (any speed <= zero)
  supercharger.speed() = -10;

  // Atempt to plan the route without setting the vehicle's max range or speed
  EXPECT_THROW(
    supercharger.PlanRoute(initial_charger_name, goal_charger_name),
    std::runtime_error
  );
}

TEST_F(SuperchargerTestFixture, TestSetPlanner)
{
  // TODO: Add this unit test once I've added a type_ member to the Planner.
}

TEST_F(SuperchargerTestFixture, TestSetOptimizer)
{
  // TODO: Add this unit test once I've added a type_ member to the Optimizer.
}