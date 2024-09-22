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
      // Suppress all terminal output
      auto old_buffer = std::cout.rdbuf(nullptr);

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

      // Define the optimization problem dimensionality and create the
      // constraint data
      n = result.route.size() - 2;
      constr_data = NLOptimizer::CreateConstraintData(result);

      // Define the initial guess, i.e. the charging duration at all nodes not
      // including the first and last nodes.
      for (auto iter = result.route.cbegin() + 1;
          iter != result.route.cend() - 1; ++iter) {
        x0.push_back(iter->duration);
      }
    }

    PlannerResult result;

    unsigned n;
    ConstraintData constr_data;
    std::vector<double> x0;
};

TEST_F(NLOptimizerTestFixture, TestGetArrivalRanges)
{
  // Compute the arrival aranges.
  const std::vector<double> arrival_ranges = 
    get_arrival_ranges(x0, &constr_data);

  // For Dijkstra's algorithm, we expect the arrival ranges to be zero for all
  // nodes from node 3 onward.
  for (size_t idx = 0; idx < arrival_ranges.size(); idx++) {
    EXPECT_DOUBLE_EQ(
      arrival_ranges.at(idx), result.route.at(idx + 2).arrival_range);
  }
}

TEST_F(NLOptimizerTestFixture, TestEqConstraint)
{
  std::vector<double> grad;
  double eq_constr_val = eq_constraint(x0, grad, &constr_data);
  EXPECT_DOUBLE_EQ(eq_constr_val, 0.0);
}

TEST_F(NLOptimizerTestFixture, TestIneqConstraintLHS)
{
  // Define the arguments
  unsigned m = n - 1;
  double* x = new double[n];
  double* grad = new double[n];
  double* result = new double[n];

  // Create the initial guess
  for ( size_t idx = 0; idx < n; idx++ ) {
    x[idx] = x0[idx];
  }
  
  // Evaluate the LHS of the inequality constraint
  ineq_constraint_lhs(
    m, result, n, const_cast<const double*>(x), grad, &constr_data);
  for ( size_t idx = 0; idx < m; idx++ ) {
    double val = result[idx];
    EXPECT_DOUBLE_EQ(result[idx], 0.0);
  }

  // Clean up
  delete x, grad, result;
}