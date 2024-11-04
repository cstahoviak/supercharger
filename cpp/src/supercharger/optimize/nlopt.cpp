/**
 * @file nloptimizer.cpp
 * @author Carl Stahoviak
 * @brief Implementation of the constrained optimization problem via NLOpt.
 * @version 0.1
 * @date 2024-09-13
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"
#include "supercharger/optimize/nlopt.h"
#include "supercharger/logging.h"

#include "nlopt.hpp"

#include <iomanip>
#include <numeric>


namespace supercharger::optimize
{
  using ConstraintData = NLOptimizer::ConstraintData;

  double objective(
    const std::vector<double>& x, std::vector<double>& grad, void* data)
  {
    // Assign the nx1 gradient vector.
    grad.assign(x.size(), 1.0);

    // Retuturn the cost: the sum of the charging durations.
    return std::reduce(x.begin(), x.end());
  }

  std::vector<double> get_arrival_ranges(
    const std::vector<double>& durations, const ConstraintData* const data)
  {
     // Compute the arrival ranges from node 3 onward.
     // TODO: Vectorize this as Ax - Ld + init_arrival_range via Eigen.
     std::vector<double> arrival_ranges(durations.size(), 0.0);
     for ( size_t idx = 0; idx < durations.size(); idx++ ) {
      // Set the previous arrival range
      double prev_arrival_range = 
        (idx) ? arrival_ranges[idx - 1] : data->init_arrival_range;

      // Compute the arrival range at the current node
      arrival_ranges[idx] = prev_arrival_range + 
        durations[idx] * data->rates[idx] - data->distances[idx];
     }

     return arrival_ranges;
  }

  void ineq_constraint_lhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data)
  {
    ConstraintData* data_ptr = reinterpret_cast<ConstraintData*>(data);
    std::vector<double> durations(x, x + n);

    // Compute the arrival ranges for all nodes [2, n]
    std::vector<double> arrival_ranges = get_arrival_ranges(durations, data_ptr);

    // Evaluate the inequality constraint function, f (f <= 0)
    std::transform(
      arrival_ranges.begin(), arrival_ranges.end(), arrival_ranges.begin(), 
      [](auto& value){ return -1 * value; });
    std::copy(arrival_ranges.begin(), arrival_ranges.end() - 1, result);

    // Assign the mxn gradient values.
    // Note that the gradient (grad) will always be valid at the first and
    // last evaluation points, but may be NULL at evaluation points in between.
    if ( grad ) {
      for ( size_t idx_m = 0; idx_m < m; idx_m++ ) {
        for ( size_t idx_n = 0; idx_n < n; idx_n++ ) {
          grad[idx_m * n + idx_n] = 
            (idx_n <= idx_m) ? -data_ptr->rates[idx_n] : 0.0;
        }
      }
    } else {
      WARN("LHS inequality constraint gradient pointer is NULL.");
    }
  }

  void ineq_constraint_rhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data)
  {
    ConstraintData* data_ptr = reinterpret_cast<ConstraintData*>(data);
    std::vector<double> durations(x, x + n);

    // Compute the arrival ranges for all nodes [2, n]
    std::vector<double> arrival_ranges = get_arrival_ranges(durations, data_ptr);

    // Evaluate the inequality constraint function, f (f <= 0)
    std::vector<double> f(m, 0.0);
    for ( size_t idx = 0; idx < m; idx++ ) {
      f[idx] = arrival_ranges[idx] - 
        (data_ptr->max_range - data_ptr->distances[idx]);
    }
    std::copy(f.begin(), f.end(), result);

    // Assign the mxn gradient values.
    // Note that the gradient (grad) will always be valid at the first and
    // last evaluation points, but may be NULL at evaluation points in between.
    if ( grad ) {
      for ( size_t idx_m = 0; idx_m < m; idx_m++ ) {
        for ( size_t idx_n = 0; idx_n < n; idx_n++ ) {
          grad[idx_m * n + idx_n] = 
            (idx_n <= idx_m) ? data_ptr->rates[idx_n] : 0.0;
        }
      }
    } else {
      WARN("RHS inequality constraint gradient pointer is NULL.");
    }
  }

  void ineq_constraint(
    std::vector<double>& result,
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data)
  {
    // Compute the arrival ranges for all nodes [2, n]
    ConstraintData* data_ptr = reinterpret_cast<ConstraintData*>(data);
    std::vector<double> arrival_ranges = get_arrival_ranges(x, data_ptr);

    // Return the arrival ranges for nodes [2, n-1]
    result.assign(arrival_ranges.cbegin(), arrival_ranges.cend() - 1);
  }

  double eq_constraint(
    const std::vector<double>& x, std::vector<double>& grad, void* data)
  {
    // Compute the arrival ranges for all nodes [2, n].
    ConstraintData* data_ptr = reinterpret_cast<ConstraintData*>(data);
    std::vector<double> arrival_ranges = get_arrival_ranges(x, data_ptr);

    // Assign the nx1 gradient values.
    grad.assign(data_ptr->rates.cbegin(), data_ptr->rates.cend());

    // Return the arrival range at the final node.
    return arrival_ranges.back();
  }

  ConstraintData NLOptimizer::CreateConstraintData(const PlannerResult& result)
  {
    // Define the optimization constraint data and the initial guess.
    ConstraintData constr_data;
    for ( auto iter = result.route.cbegin() + 1; iter != result.route.cend(); ++iter )
    {
      const Node& current = *iter;
      const Node& previous = *(iter - 1);
      constr_data.distances.push_back(math::distance(previous, current));
      constr_data.rates.push_back(current.charger().rate);
    }
    constr_data.init_arrival_range = result.route.at(1).arrival_range;
    constr_data.max_range = result.max_range;

    // Remove values that do not affect the optimization, i.e. the distance 
    // between the first and second nodes, and final node's charging rate.
    constr_data.distances.erase(constr_data.distances.begin());
    constr_data.rates.pop_back();

    return constr_data;
  }

  PlannerResult NLOptimizer::Optimize(const PlannerResult& result) const
  {
    // Define the optimization algorithm and dimensionality. Note that the 
    // algorithm and dimension parameters of the object are immutable (cannot be
    // changed without creating a new object), but are queryable.
    size_t dim = result.route.size() - 2;
    nlopt::opt opt(algorithm_, dim);

    // Create the optimization constraint data.
    ConstraintData constr_data = NLOptimizer::CreateConstraintData(result);

    // Set the lower bounds on the charging durations (all zeros).
    std::vector<double> lb = std::vector<double>(dim, 0.0);
    opt.set_lower_bounds(lb);

    // Set the upper bounds on the charging durations.
    std::vector<double> ub = std::vector<double>(dim, HUGE_VAL);
    size_t idx{0};
    for (const double& rate : constr_data.rates ) {
      ub[idx] = 
        (result.max_range - result.route.at(idx + 1).arrival_range) / rate;
      idx++;
    }
    opt.set_upper_bounds(ub);

    // Set the objective function.
    opt.set_min_objective(objective, nullptr);

    // Add the inequality constraints on the arrival ranges, [3, N-1].
    unsigned m = dim - 1;
    double tol = 1e-9;
    const std::vector<double> mtol = std::vector<double>(m, tol);
    opt.add_inequality_mconstraint(ineq_constraint_lhs, &constr_data, mtol);
    opt.add_inequality_mconstraint(ineq_constraint_rhs, &constr_data, mtol);

    // Add an equality constraint for the arrival range at the last node.
    opt.add_equality_constraint(eq_constraint, &constr_data, tol);

    // Set a relative tolerance for the optimization parameters.
    opt.set_xtol_rel(1e-8);

    // Set the initial guess, i.e. the charging duration at all nodes not
    // including the first and last nodes.
    // TODO: Do this better (the const_cast here feels pretty ugly).
    std::vector<double> x(
      const_cast<PlannerResult&>(result).durations().cbegin() + 1,
      const_cast<PlannerResult&>(result).durations().cend() - 1);

    // Run the optimization!
    double minf{0};
    try {
      DEBUG("Optimizing via NLOpt.");
      nlopt::result new_durations = opt.optimize(x, minf);
      DEBUG("NLOpt found minimum at f(x) = " << std::setprecision(10) << minf);
    }
    catch( std::exception &e ) {
      INFO("NLOpt failed: " << e.what());
    }

    return CreateOptimizedResult_(result, x);
  }

  PlannerResult NLOptimizer::CreateOptimizedResult_(
    const PlannerResult& result, const std::vector<double>& new_durations) const
  {
    using namespace supercharger::algorithm;

    // Create the new result as a copy of the original.
    PlannerResult optimized = result;

    // Update the charging duration for all nodes between the start and end.
    for ( size_t idx = 0; idx < new_durations.size(); idx++ ) {
      optimized.route[idx + 1].duration = new_durations[idx];
    }

    // Update the arrival and departure ranges and the cost at each node.
    for ( auto iter = optimized.route.begin() + 1;
          iter != optimized.route.end();
          ++iter )
    {
      const Node& previous = *(iter - 1);
      Node& current = *iter;

      // Update the arrival range at the current node.
      current.arrival_range = GetArrivalRange(previous, current);

      // Update the cost at the current node.
      current.cost = previous.cost + previous.duration + \
        math::distance(previous, current) / result.speed;

      // For all nodes but the final node, update the departure range.
      if ( iter != optimized.route.end() - 1 ) {
        current.departure_range = GetDepartureRange(current);
      }
    }

    // Finally, update the total cost of the route.
    optimized.cost = optimized.route.back().cost;
    return optimized;
  }
} // end namespace supercharger::optimize