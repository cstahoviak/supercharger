/**
 * @file optimizer.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for the Optimizer (and Optimizer-derived) classes.
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/optimize/naive.h"
#include "supercharger/optimize/nlopt.h"

#include <pybind11/pybind11.h>

namespace supercharger
{
  using PlannerResult = algorithm::PlannerResult;

  namespace optimize
  {
    /**
     * @brief The PyOptimizer "trampline" class allows the Optimizer class to
     * be extensible on the python side.
     */
    class PyOptimizer : public Optimizer
    {
      public:
        // Inherit the constructor(s)
        using optimize::Optimizer::Optimizer;

        // "Trampoline" function (required for each virtual function)
        PlannerResult Optimize(const PlannerResult&) const override;
    };

    PlannerResult PyOptimizer::Optimize(
      const PlannerResult& result) const
    {
      PYBIND11_OVERRIDE_PURE(
        PlannerResult,  // Return type
        Optimizer,      // Parent class
        Optimize,       // Name of C++ function (must match python name)
        result          // Argument(s)
      );
    }
  } // end namespace supercharger::optimizer
} // end namespace supercharger

namespace py = pybind11;
using namespace supercharger::optimize;

void initOptimizer(py::module_& m)
{
  py::enum_<Optimizer::OptimizerType>(m, "OptimizerType")
    .value("NAIVE", Optimizer::OptimizerType::NAIVE)
    .value("NLOPT", Optimizer::OptimizerType::NLOPT)
    .value("NONE", Optimizer::OptimizerType::NONE)
    .export_values()
  ;

  // Note that the "trampoline" class PyOptimizer is included as a template argument.
  py::class_<Optimizer, PyOptimizer>(m, "Optimizer")
    .def(py::init<>())
    .def_static("get_optimizer", &Optimizer::GetOptimizer, py::arg("type"))
    .def("optimize", &Optimizer::Optimize, py::arg("result"))
  ;

  py::class_<NaiveOptimizer, Optimizer>(m, "NaiveOptimizer")
    .def(py::init<>())
  ;

  py::class_<NLOptimizer, Optimizer>(m, "NLOptimizer")
    .def(py::init<>())
  ;
}