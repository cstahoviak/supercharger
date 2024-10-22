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
// Need to include the funcional header in any translation unit where 
// DijkstrasCostFcnType appears.
#include <pybind11/functional.h>
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
    // Bindings for "top-level" Supercharger ctor intentionally omitted.
    .def(py::init<NaiveCostFcnType>(), py::arg("naive_cost_type"))
    .def(py::init<NaiveCostFcnType, OptimizerType>(),
      py::arg("naive_cost_type"), py::arg("optim_type"))
    .def(py::init<DijkstrasCostFcnType>(), py::arg("cost_f"))
    .def(py::init<DijkstrasCostFcnType, OptimizerType>(),
      py::arg("cost_f"), py::arg("optim_type"))

    .def("plan_route", &Supercharger::PlanRoute,
      py::arg("origin"), py::arg("destination"))

    .def("set_planner",
      py::overload_cast<NaiveCostFcnType>(&Supercharger::SetPlanner),
      py::arg("naive_cost_type"))
    .def("set_planner",
      py::overload_cast<DijkstrasCostFcnType>(&Supercharger::SetPlanner),
      py::arg("cost_f"))
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