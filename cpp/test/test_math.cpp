/**
 * @file test_math.cpp
 * @author your name (you@domain.com)
 * @brief Unit tests for math functions
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/math/math.h"

#include <gtest/gtest.h>

using namespace supercharger::math;

TEST(TestMath, TestGreatCircleDistance) {
  // Define the lat/lon of the locations being tested
  double worthington_mn_lat = 43.63385;
  double worthington_mn_lon = -95.595647;

  double oakdale_mn_lat = 44.964892;
  double oakdale_mn_lon = -92.961249;

  // Compute the "great circle" math::distance
  double dist = great_circle_distance(
    worthington_mn_lat, worthington_mn_lon, oakdale_mn_lat, oakdale_mn_lon);

  // Expected result comes from evaluating the nested Matlab function call:
  // deg2km(distance(43.63385,-95.595647,44.964892,-92.961249), 6356.752)
  const double expected = 256.0343746070473;
  EXPECT_DOUBLE_EQ(dist, expected);
}