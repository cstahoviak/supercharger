#pragma once
/**
 * @file nloptimizer.h
 * @author Carl Stahoviak
 * @brief Implements route optimization via the NLOpt library.
 * @version 0.1
 * @date 2024-09-13
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/optimize/optimizer.h"

#include "nlopt.hpp"


namespace supercharger::optimize
{
  class NLOptimizer : public Optimizer
  {
    public:
      /**
       * @brief Stores all data required by the constraint functions.
       */
      struct ConstraintData
      {
        std::vector<double> distances;
        std::vector<double> rates;
        double init_arrival_range{0};
        double max_range{0};
      };

      /**
       * @brief Creates the constraint data (the distances and rates) from the
       * provided planner result.
       * 
       * @param result 
       * @return ConstraintData 
       */
      static ConstraintData CreateConstraintData(const PlannerResult&);

      /**
       * @brief Optimizes the provided route via a constrained optimization
       * scheme.
       * 
       * @param result The "unoptimized" reference route.
       * @return PlannerResult The oprimized route.
       */
      PlannerResult Optimize(const PlannerResult&) const override;

    private:
      /**
       * @brief Creates the optimized route from the reference planner result
       * and the newly optimized set of charging durations.
       * 
       * @param result The reference result.
       * @param new_durations The optimized set of charging durations at all
       * nodes not including the first and last nodes.
       * @return PlannerResult The optimized planner result.
       */
      PlannerResult CreateOptimizedResult_(
        const PlannerResult&, const std::vector<double>&) const;

      // SLSQP performs local, gradient-based constrained optimization.
      nlopt::algorithm algorithm_ = nlopt::LD_SLSQP;
  };

  /**
   * @brief The nonlinear optimization cost function.
   * 
   * @param x A vector of charging durations for all nodes in the route except
   *  the first and last nodes.
   * @param grad The gradient, i.e. the partial derivatives of the objective
   *  function with repect to x. The gradient is only needed for gradient-based
   *  algorithms; if you use a derivative-free optimization algorithm, grad will
   *  always be NULL and computing the gradient is not required.
   * @param data A pointer to a ConstraintData instance.
   * @return double The sum of the charging durations.
   */
  double objective(
    const std::vector<double>& x,
    std::vector<double>& data,
    void*
  );

  std::vector<double> get_arrival_ranges(
    const std::vector<double>& durations,
    const NLOptimizer::ConstraintData* const data
  );

  /**
   * @brief The nonlinear optimization constraint function that complies with
   * the NLOpt mfunc function type. Note that the C++ style "vfunc" function
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
   * @param x The state vector (the charging durations) at the current iteration.
   * @param grad A pointer to the gradient. The gradient is an array of length
   *  mxn. The n dimension of grad is stored contiguously, so that ∂c_i/∂x_j
   *  is stored in grad[i*n + j].
   * @param data A pointer to a ConstraintData instance.
   */
  void ineq_constraint_lhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data
  );

  /**
   * @brief The nonlinear optimization constraint function that complies with
   * the NLOpt mfunc function type. Note that the C++ style "vfunc" function
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
   * @param result The constraint vector.
   * @param n The dimension of the parameter space.
   * @param x The state vector (the charging durations) at the current iteration.
   * @param grad A pointer to the gradient. The gradient is an array of length
   *  mxn. The n dimension of grad is stored contiguously, so that ∂c_i/∂x_j
   *  is stored in grad[i*n + j].
   * @param data A pointer to a ConstraintData instance.
   */
  void ineq_constraint_rhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data
  );

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
   * @param data A pointer to a ConstraintData instance.
   */
  void ineq_constraint(
    std::vector<double>& result,
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
  );

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
   * @param data A pointer to a ConstraintData instance.
   * @return std::vector<double> The equality constraint function value.
   */
  double eq_constraint(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
  );
} // end namespace supercharger