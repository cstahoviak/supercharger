/**
 * @file planner.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for...
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "planner.h"

#include <pybind11/pybind11.h>
// Required for automatic type conversions between python lists and std::vector
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace supercharger;

void initRoutePlanner(py::module_& m)
{
  // string stream operator
  // m.def("__repr__", 
  //   py::overload_cast<std::ostream&, const std::vector<Node>&>(&operator<<));
  // m.def("__repr__", 
  //   py::overload_cast<std::ostream&, const std::vector<Node*>&>(&operator<<));

  py::class_<RoutePlanner>(m, "RoutePlanner")
    .def(py::init<AlgoType, CostFcnType>(), py::arg("algo_type"), py::arg("cost_type"))
    // Since cost_type is a default argument, we need to define a seperate 
    // constructor that leaves out the CostFcnType parameter.
    .def(py::init<AlgoType>(), py::arg("algo_type"))
    .def("plan_route", &RoutePlanner::PlanRoute, py::arg("origin"), py::arg("destination"))
    .def("optimize_route", &RoutePlanner::OptimizeRoute, py::arg("result"))
    .def_property_readonly("network", &RoutePlanner::network)
    .def_property_readonly("destination", &RoutePlanner::destination)
    // Require a lambda to deal with getters and setters of the same name.
    .def_property("max_range",
      [](const RoutePlanner& self) { return self.max_range(); }, 
      [](RoutePlanner& self, double value) { self.max_range() = value; }
    )
    .def_property("speed",
      [](const RoutePlanner& self) { return self.speed(); }, 
      [](RoutePlanner& self, double value) { self.speed() = value; }
    )
  ;

}