/**
 * @file nloptimizer.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-13
 * 
 * @copyright Copyright (c) 2024
 */

#include "supercharger/optimize/nlopt.h"
#include "supercharger/logging.h"

#include "nlopt.hpp"

#include <iomanip>
#include <numeric>

namespace supercharger::optimize
{
  using ConstraintData = NLOptimizer::ConstraintData;

  /**
   * @brief The nonlinear optimization cost function.
   * 
   * @param x A vector of charging durations for all nodes in the route except
   *  the first and last nodes.
   * @param grad The gradient, i.e. the partial derivatives of the objective
   *  function with repect to x. The gradient is only needed for gradient-based
   *  algorithms; if you use a derivative-free optimization algorithm, grad will
   *  always be NULL and you need never compute any derivatives.
   * @param data A pointer to am OptimizerData instance.
   * @return double The sum of the charging durations.
   */
  double cost_fcn(
    const std::vector<double>& x, std::vector<double>& grad, void* data)
  {
    // Assign the nx1 gradient vector.
    grad.assign(x.size(), 1.0);

    // Retuturn the cost - the sum of the charging durations.
    return std::reduce(x.begin(), x.end());
  }

  std::vector<double> get_arrival_ranges(
    const std::vector<double>& durations, const ConstraintData* const data)
  {
     // Compute the arrival ranges from node 3 onward.
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

    /**
   * @brief The nonlinear optimization constraint function that complies with
   * the NLOpt mfunc function type. Not the at the C++ style "vfunc" function
   * signature is not currently supported for vector-valued constraint
   * functions. The constraint function is defined as:
   * 
   * 0 <= arrival_range[n] <= MAX_RANGE - dist(n, n-1)
   * 
   * where dist(n, n-1) is the distance between node n and the previous node.
   * This left hand side of this constraint can be re-written as:
   * 
   * -arrival_range[n] <= 0
   * 
   * @param m The dimension of the contraint vector.
   * @param result The constraint vector.
   * @param n The dimension of the parameter space.
   * @param x The 
   * @param grad A pointer to the gradient. The gradient is an array of length
   *  mxn. The n dimension of grad is stored contiguously, so that ∂c_i/∂x_j
   *  is stored in grad[i*n + j].
   * @param data 
   */
  void ineq_constraint_lhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data)
  {
    // Cast pointers to other types
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

    return;
  }

  /**
   * @brief The nonlinear optimization constraint function that complies with
   * the NLOpt mfunc function type. Not the at the C++ style "vfunc" function
   * signature is not currently supported for vector-valued constraint
   * functions. The constraint function is defined as:
   * 
   * 0 <= arrival_range[n] <= MAX_RANGE - dist(n, n-1)
   * 
   * where dist(n, n-1) is the math::distance between node n and the previous node.
   * This right hand side of this constraint can be re-written as:
   * 
   * arrival_range[n] - (MAX_RANGE - dist(n, n-1)) <= 0
   * 
   * @param m The dimension of the contraint vector.
   * @param result The constraint vector.
   * @param n 
   * @param x 
   * @param grad 
   * @param data 
   */
  void ineq_constraint_rhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data)
  {
    // Cast pointers to other types
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
    if ( grad ) {
      for ( size_t idx_m = 0; idx_m < m; idx_m++ ) {
        for ( size_t idx_n = 0; idx_n < n; idx_n++ ) {
          grad[idx_m * n + idx_n] = 
            (idx_n <= idx_m) ? data_ptr->rates[idx_n] : 0.0;
        }
      }
    } else {
      WARN("LHS inequality constraint gradient pointer is NULL.");
    }

    return;
  }

  /**
   * @brief (UNUSED) The inequaility constraint function. The inequality
   * constraint applies to all nodes between the second node and the second to
   * last node, [2, n-1].
   * 
   * Note that NLOpt does not support a vector-valued inequality constraint
   * function with a C++-style function signature. This function is unused.
   * 
   * Note that NLOpt expects constraints to be of the form:
   * constraint_fcn(x) <= 0.
   * 
  * @param x A vector of charging durations for all nodes in the route except
   *  the first and last nodes.
   * @param grad The gradient, i.e. the partial derivatives of the objective
   *  function with repect to x. The gradient is only needed for gradient-based
   *  algorithms; if you use a derivative-free optimization algorithm, grad will
   *  always be NULL and you need never compute any derivatives.
   * @param data A pointer to am OptimizerData instance.
   * @return std::vector<double> 
   */
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

  /**
   * @brief The equality constraint function. The equality constraint applies
   * to the last node - a time optimal route will have an arrival range of zero
   * at the final node.
   * 
   * Note that NLOpt expects constraints to be of the form:
   * constraint_fcn(x) <= 0.
   * 
  * @param x A vector of charging durations for all nodes in the route except
   *  the first and last nodes.
   * @param grad The gradient, i.e. the partial derivatives of the objective
   *  function with repect to x. The gradient is only needed for gradient-based
   *  algorithms; if you use a derivative-free optimization algorithm, grad will
   *  always be NULL and you need never compute any derivatives.
   * @param data A pointer to am OptimizerData instance.
   * @return std::vector<double> 
   */
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

  /**
   * @brief Creates the constraint data (the distances and rates) from the
   * provided planner result.
   * 
   * @param result 
   * @return ConstraintData 
   */
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

  /**
   * @brief Optimizes the provided route via a constrained optimization scheme.
   * 
   * @param result The "unoptimized" reference route.
   * @return PlannerResult The oprimized route.
   */
  PlannerResult NLOptimizer::Optimize(const PlannerResult& result) const
  {
    // Define the optimization algorithm and dimensionality.
    size_t dim = result.route.size() - 2;
    nlopt::opt opt(algorithm_, dim);

    // Define the optimization constraint data.
    ConstraintData constr_data = NLOptimizer::CreateConstraintData(result);

    // Set the lower bounds on the charging durations (zero).
    std::vector<double> lb = std::vector<double>(dim, 0.0);
    opt.set_lower_bounds(lb);

    // Set the upper bounds on the charging rates.
    std::vector<double> ub = std::vector<double>(dim, HUGE_VAL);
    size_t idx{0};
    for (const double& rate : constr_data.rates ) {
      ub[idx] = 
        (result.max_range - result.route.at(idx + 1).arrival_range) / rate;
      idx++;
    }
    opt.set_upper_bounds(ub);

    // Set the objective function.
    opt.set_min_objective(cost_fcn, nullptr);

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

  /**
   * @brief Creates the optimized route from the reference planner result and
   * the newly optimized set of charging durations.
   * 
   * @param result The reference result.
   * @param new_durations The optimized set of charging durations at all nodes
   * not including the first and last nodes.
   * @return PlannerResult The optimized planner result.
   */
  PlannerResult NLOptimizer::CreateOptimizedResult_(
    const PlannerResult& result, const std::vector<double>& new_durations) const
  {
    // Create the new result as a copy of the original.
    PlannerResult optimized = result;

    // Update the charging duration for all nodes between the start and end.
    for ( size_t idx = 0; idx < new_durations.size(); idx++ ) {
      optimized.route[idx + 1].duration = new_durations[idx];
    }

    // Update the arrival and departure ranges and the cost at each node.
    for ( auto iter = optimized.route.begin() + 1; iter != optimized.route.end(); ++iter )
    {
      const Node& previous = *(iter - 1);
      Node& current = *iter;

      // Update the arrival range at the current node.
      // TODO: If ComputeArrivalRange is a free function (decoupled from the
      // Planner), I could use it here.
      current.arrival_range = previous.arrival_range + \
        previous.duration * previous.charger().rate - \
        math::distance(previous, current);

      // Update the cost at the current node
      current.cost = previous.cost + previous.duration + \
        math::distance(previous, current) / result.speed;

      // For all nodes but the final node, update the departure range.
      // TODO: If ComputeDepartureRange is a free function (decoupled from the
      // Planner), I could use it here.
      if ( iter != optimized.route.end() - 1 ) {
        current.departure_range = current.arrival_range + \
          current.duration * current.charger().rate;
      }
    }

    // Finally, update the total cost of the route.
    optimized.cost = optimized.route.back().cost;
    return optimized;
  }
} // end namespace supercharger::optimize