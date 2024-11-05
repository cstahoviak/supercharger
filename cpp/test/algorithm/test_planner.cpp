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

using namespace supercharger::algorithm;


TEST(TestPlanner, TestGetInvalidPlanner)
{
  // Ensure that attempting to create a "Naive" planner with a custom
  // cost function fails.
  EXPECT_THROW(
    Planner::GetPlanner(Planner::AlgorithmType::NAIVE, SimpleCost),
    std::invalid_argument
  );
}