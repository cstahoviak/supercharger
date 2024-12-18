/**
 * @file test_nloptimizer.cpp
 * @author Carl Stahoviak
 * @brief Unit tests for the NLOptimizer class.
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/optimize/nlopt.h"

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

      // Define some vehicle constants
      const double max_range{320};
      const double speed{105};

      // Create the route planner
      auto planner = algorithm::Dijkstras(algorithm::SimpleCost);

      // Plan the route
      planner_result = planner.Plan(
        initial_charger_name, goal_charger_name, max_range, speed);

      // Define the optimization problem dimensionality and create the
      // constraint data.
      n = planner_result.route.size() - 2;
      constr_data = NLOptimizer::CreateConstraintData(planner_result);

      // Define the initial guess, i.e. the charging duration at all nodes not
      // including the first and last nodes.
      x0.assign(planner_result.durations().cbegin() + 1,
        planner_result.durations().cend() - 1);
    }

    // Store the result of Dijkstra's planning algorithm.
    PlannerResult planner_result;

    // Store data related to the optimization problem.
    unsigned n;
    ConstraintData constr_data;
    std::vector<double> x0;

    // The expected charging durations for the optimized route.
    std::vector<double> durations {
      0.0,
      2.4854182205752964,
      0.3780665153428539,
      0.6986804715671957,
      2.3188405797101415,
      0.52965134512121326,
      0.0
    };
};

TEST_F(NLOptimizerTestFixture, TestGetArrivalRanges)
{
  // Compute the arrival aranges.
  const std::vector<double> arrival_ranges = 
    get_arrival_ranges(x0, &constr_data);

  // For Dijkstra's algorithm, we expect the arrival ranges to be zero for all
  // nodes from node 3 onward.
  constexpr int offset{2};
  for (size_t idx = 0; idx < arrival_ranges.size(); idx++) {
    EXPECT_DOUBLE_EQ(arrival_ranges.at(idx),
      planner_result.route.at(idx + offset)->arrival_range);
  }
}

TEST_F(NLOptimizerTestFixture, TestCostFcn)
{
  std::vector<double> charging_times;
  double total_charging_time{0};
  for ( const std::shared_ptr<const Node>& node : planner_result.route ) {
    charging_times.push_back(node->duration);
    total_charging_time += node->duration;
  }

  std::vector<double> grad;
  EXPECT_DOUBLE_EQ(
    objective(charging_times, grad, nullptr), total_charging_time);
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

TEST_F(NLOptimizerTestFixture, TestOptimizeRoute)
{
  // Create the optimizer.
  const auto optimizer =
    Optimizer::GetOptimizer(Optimizer::OptimizerType::NLOPT);

  // Optimize the route.
  PlannerResult optimized_result = optimizer->Optimize(planner_result);

  // Verify that some result information has remained unchanged.
  EXPECT_EQ(optimized_result.route.size(), planner_result.route.size());
  EXPECT_DOUBLE_EQ(optimized_result.max_range, planner_result.max_range);
  EXPECT_DOUBLE_EQ(optimized_result.speed, planner_result.speed);

  // Verify that the route cost has either improved or remained the same.
  EXPECT_LE(optimized_result.cost, planner_result.cost);

  // Verify that the nodes and charging durations are correct. 
  for ( size_t idx = 0; idx < optimized_result.route.size(); idx++ ) {
    EXPECT_EQ(optimized_result.route[idx]->name(),
      planner_result.route[idx]->name());
    EXPECT_DOUBLE_EQ(optimized_result.route[idx]->duration, durations[idx]);
  }
}