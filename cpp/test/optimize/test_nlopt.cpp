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
#include "supercharger/optimize/nlopt.h"
#include "supercharger/planner.h"

#include <gtest/gtest.h>

using namespace supercharger;
using namespace supercharger::optimize;
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
      RoutePlanner planner(AlgoType::DIJKSTRAS);

      // Set the vehicle's speed and max range
      planner.max_range() = 320;
      planner.speed() = 105;

      // Plan the route
      planner_result = planner.PlanRoute(
        initial_charger_name,
        goal_charger_name
      );

      // Define the optimization problem dimensionality and create the
      // constraint data
      n = planner_result.route.size() - 2;
      constr_data = NLOptimizer::CreateConstraintData(planner_result);

      // Define the initial guess, i.e. the charging duration at all nodes not
      // including the first and last nodes.
      for ( auto iter = planner_result.route.cbegin() + 1;
          iter != planner_result.route.cend() - 1; ++iter ) {
        x0.push_back(iter->duration);
      }
    }

    // Store the result of Dijkstra's planning algorithm.
    PlannerResult planner_result;

    // Store data related to the optimization problem.
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
      arrival_ranges.at(idx), planner_result.route.at(idx + 2).arrival_range);
  }
}

TEST_F(NLOptimizerTestFixture, TestCostFcn)
{
  std::vector<double> charging_times;
  double total_charging_time{0};
  for ( const Node& node : planner_result.route ) {
    charging_times.push_back(node.duration);
    total_charging_time += node.duration;
  }

  std::vector<double> grad;
  EXPECT_DOUBLE_EQ(
    cost_fcn(charging_times, grad, nullptr), total_charging_time);
}

TEST_F(NLOptimizerTestFixture, TestEqConstraint)
{
  std::vector<double> grad;

  // Validate the scalar value returned by the equality constraint.
  EXPECT_DOUBLE_EQ(eq_constraint(x0, grad, &constr_data), 0.0);

  // Validate that the gradient is being set correctly.
  for ( size_t idx = 0; idx < x0.size(); idx++ ) {
    EXPECT_DOUBLE_EQ(grad.at(idx), constr_data.rates.at(idx));
  }
}

TEST_F(NLOptimizerTestFixture, TestIneqConstraintLHS)
{
  // Define the arguments
  unsigned m = n - 1;
  double* x = new double[n];
  double* grad = new double[m * n];
  double* result = new double[n];

  // Create the initial guess
  for ( size_t idx = 0; idx < n; idx++ ) {
    x[idx] = x0[idx];
  }
  
  // Evaluate the LHS of the inequality constraint
  ineq_constraint_lhs(
    m, result, n, const_cast<const double*>(x), grad, &constr_data);
  for ( size_t idx = 0; idx < m; idx++ ) {
    EXPECT_DOUBLE_EQ(result[idx], 0.0);
  }

  // TODO: Validate that the gradient is being set correctly.

  // Clean up
  delete[] x, grad, result;
}

TEST_F(NLOptimizerTestFixture, TestIneqConstraintRHS)
{
  // Define the arguments
  unsigned m = n - 1;
  double* x = new double[n];
  double* grad = new double[m * n];
  double* result = new double[n];

  // Create the initial guess
  for ( size_t idx = 0; idx < n; idx++ ) {
    x[idx] = x0[idx];
  }

  // Define the expected result
  std::vector<double> expected(m, 0.0);
  for ( size_t idx = 0; idx < m; idx++ ) {
    expected[idx] = -(planner_result.max_range - constr_data.distances[idx]);
  }
 
  // Evaluate the LHS of the inequality constraint
  ineq_constraint_rhs(
    m, result, n, const_cast<const double*>(x), grad, &constr_data);
  for ( size_t idx = 0; idx < m; idx++ ) {
    double val = result[idx];
    EXPECT_DOUBLE_EQ(result[idx], expected[idx]);
  }

  // TODO: Validate that the gradient is being set correctly.

  // Clean up
  delete[] x, grad, result;
}