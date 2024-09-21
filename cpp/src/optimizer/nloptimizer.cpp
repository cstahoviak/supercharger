/**
 * @file nloptimizer.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-13
 * 
 * @copyright Copyright (c) 2024
 * 
 * TODO: Fix the "free(): invalid size" error.
 *  1. Implement the equality constraint via a C-style function.
 *  2. Completely switch to the C-style implementation.
 */

#include "optimizer/nloptimizer.h"
#include "logging.h"

#include "nlopt.hpp"

#include <iomanip>
#include <numeric>

namespace supercharger
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
   * @param grad 
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
    std::vector<double> durations(x, x + sizeof(x) / sizeof(x[0]));

    // Compute the arrival ranges for all nodes [2, n]
    std::vector<double> arrival_ranges = get_arrival_ranges(durations, data_ptr);

    // Evaluate the inequality constraint function, f (f <= 0)
    std::transform(
      arrival_ranges.begin(), arrival_ranges.end(), arrival_ranges.begin(), 
      [](auto& value){return -1 * value;});
    std::copy(arrival_ranges.begin(), arrival_ranges.end() - 1, result);
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
   * where dist(n, n-1) is the distance between node n and the previous node.
   * This right hand side of this constraint can be re-written as:
   * 
   * arrival_range[n] - (MAX_RANGE - dist(n, n-1)) <= 0
   * 
   * @param m The dimension of the contraint vector.
   * @param result The constraint 
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
    std::vector<double> durations(x, x + sizeof(x) / sizeof(x[0]));

    // Compute the arrival ranges for all nodes [2, n]
    std::vector<double> arrival_ranges = get_arrival_ranges(durations, data_ptr);

    // Evaluate the inequality constraint function, f (f <= 0)
    std::vector<double> f(arrival_ranges.size(), 0.0);
    for ( size_t idx = 0; idx < m; idx++ ) {
      f[idx] = arrival_ranges[idx] - 
        (data_ptr->max_range - data_ptr->distances[idx]);
    }
    std::copy(f.begin(), f.end() - 1, result);
    return;
  }

  /**
   * @brief The inequaility constraint function. The inequality constraint
   * applies to all nodes between the second node and the second to last node, 
   * [2, n-1].
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
   * @brief The equaility constraint function. The equality constraint applies
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

    // Return the arrival range at the final node.
    return arrival_ranges.back();
  }

  PlannerResult NLOptimizer::Optimize(const PlannerResult& result) const
  {
    // Define the optimization algorithm and dimensionality.
    size_t dim = result.route.size() - 2;
    nlopt::opt opt(nlopt::LN_AUGLAG, dim);

    // Define the optimization constraint data and the initial guess.
    ConstraintData constr_data;
    std::vector<double> x;
    for ( auto iter = result.route.cbegin() + 1; iter != result.route.cend(); ++iter )
    {
      const Node& current = *iter;
      const Node& previous = *(iter - 1);
      constr_data.distances.push_back(distance(previous, current));
      constr_data.rates.push_back(current.charger().rate);
      x.push_back(current.duration);
    }
    constr_data.init_arrival_range = result.route.at(1).arrival_range;
    constr_data.max_range = result.max_range;

    // Remove values that do not affect the optimization, i.e. the distance 
    // between the first and second nodes, and final node's charging rate.
    constr_data.distances.erase(constr_data.distances.begin());
    constr_data.rates.pop_back();
    x.pop_back();

    // Set the lower bounds on the charging rates.
    std::vector<double> lb = std::vector<double>(dim, 0.0);
    opt.set_lower_bounds(lb);

    // Set the upper bounds on the charging rates.
    std::vector<double> ub = std::vector<double>(dim, HUGE_VAL);
    size_t idx {0};
    for (const double& rate : constr_data.rates ) {
      ub[idx] = 
        (result.max_range - result.route.at(idx + 1).arrival_range) / rate;
      idx++;
    }
    opt.set_upper_bounds(ub);

    // Set the objective function
    opt.set_min_objective(cost_fcn, NULL);

    // Add the inequality constraints on the arrival ranges, [3, N-1].
    unsigned m = dim - 1;
    double tol = 1e-9;
    const std::vector<double> mtol = std::vector<double>(m, tol);
    opt.add_inequality_mconstraint(ineq_constraint_lhs, &constr_data, mtol);
    opt.add_inequality_mconstraint(ineq_constraint_rhs, &constr_data, mtol);

    // Add an equality constraint for the arrival range at the last node
    opt.add_equality_constraint(eq_constraint, &constr_data, tol);

    // Set a relative tolerance for the optimization parameters
    opt.set_xtol_rel(1e-4);

    // Run the optimization!
    double minf{0};
    try {
      DEBUG("Optimizing via NLOpt.");
      nlopt::result new_durations = opt.optimize(x, minf);
      INFO("Found minimum at f(x) = " << std::setprecision(10) << minf);
    }
    catch( std::exception &e ) {
      INFO("nlopt failed: " << e.what());
    }

    return result;
  }
}