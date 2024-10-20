/**
 * @file supercharger.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for the Supercharger class.
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/supercharger.h"

#include <pybind11/pybind11.h>
// Required for automatic type conversions between python lists and std::vector
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace supercharger;

void initSupercharger(py::module_& m)
{
  // string stream operator
  // m.def("__repr__", 
  //   py::overload_cast<std::ostream&, const std::vector<Node>&>(&operator<<));
  // m.def("__repr__", 
  //   py::overload_cast<std::ostream&, const std::vector<Node*>&>(&operator<<));

  py::class_<Supercharger>(m, "Supercharger")
    .def(py::init<AlgoType, CostFcnType, OptimizerType>(),
      py::arg("algo_type"), py::arg("cost_type"), py::arg("optim_type"))
    .def(py::init<AlgoType, OptimizerType>(),
      py::arg("algo_type"), py::arg("optim_type"))
    .def(py::init<AlgoType, CostFcnType>(),
      py::arg("algo_type"), py::arg("cost_type"))
    .def(py::init<AlgoType>(), py::arg("algo_type"))

    .def("plan_route", &Supercharger::PlanRoute,
      py::arg("origin"), py::arg("destination"))
    .def("set_planning_algorithm",
      py::overload_cast<AlgoType, CostFcnType>(&Supercharger::SetPlanningAlgorithm),
      py::arg("algo_type"), py::arg("cost_type"))
    .def("set_planning_algorithm",
      py::overload_cast<AlgoType>(&Supercharger::SetPlanningAlgorithm),
      py::arg("algo_type"))
    .def("set_optimizer", &Supercharger::SetOptimizer, py::arg("type"))

    .def_property_readonly("network", &Supercharger::network)
    .def_property_readonly("destination", &Supercharger::destination)
    // Require a lambda to deal with getters and setters of the same name.
    .def_property("max_range",
      [](const Supercharger& self) { return self.max_range(); }, 
      [](Supercharger& self, double value) { self.max_range() = value; }
    )
    .def_property("speed",
      [](const Supercharger& self) { return self.speed(); }, 
      [](Supercharger& self, double value) { self.speed() = value; }
    )
  ;
}