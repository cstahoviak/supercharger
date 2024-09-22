#pragma once
/**
 * @file nloptimizer.h
 * @author Carl Stahoviak
 * @brief 
 * @version 0.1
 * @date 2024-09-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "optimizer/optimizer.h"

namespace supercharger
{
  class NLOptimizer : public Optimizer
  {
    public:
      struct ConstraintData
      {
        std::vector<double> distances;
        std::vector<double> rates;
        double init_arrival_range{0};
        double max_range{0};
      };

      static ConstraintData CreateConstraintData(const PlannerResult&);
      PlannerResult Optimize(const PlannerResult&) const override;
  };

  double cost_fcn(
    const std::vector<double>& x,
    std::vector<double>& data,
    void*
  );

  std::vector<double> get_arrival_ranges(
    const std::vector<double>& durations,
    const NLOptimizer::ConstraintData* const data
  );

  void ineq_constraint_lhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data
  );

  void ineq_constraint_rhs(
    unsigned m,
    double* result,
    unsigned n,
    const double* x,
    double* grad,
    void* data
  );

  void ineq_constraint(
    std::vector<double>& result,
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
  );

  double eq_constraint(
    const std::vector<double>& x,
    std::vector<double>& grad,
    void* data
  );
} // end namespace supercharger