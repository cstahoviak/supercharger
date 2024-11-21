/**
 * @file types.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for the custom Supercharger types.
 * @version 0.1
 * @date 2024-11-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/types.h"

#include <pybind11/pybind11.h>
// NOTE: required because PlannerResult::route is a std::vector.
// NOTE: When you include any of the optional casters (pybind11/eigen.h, 
// pybind11/stl.h, etc.), you need to include them consistently in every 
// translation unit.
// #include <pybind11/stl.h>

#include <sstream>

namespace py = pybind11;
using namespace supercharger;


std::string to_string(const PlannerResult& result) {
  std::ostringstream os;
  size_t idx = 0;
  for ( const std::shared_ptr<const Node>& node : result.route ) {
    os << *node;
    if ( idx < result.route.size() - 1 ) {
        os << ", ";
      }
      idx++;
  }
  return os.str();
}


void initTypes(py::module_& m)
{
  py::class_<PlannerResult>(m, "PlannerResult")
    .def(py::init<>())
    .def(py::init<std::vector<std::shared_ptr<Node>>, double, double, double>(),
      py::arg("route"), py::arg("cost"), py::arg("max_range"), py::arg("speed"))
    .def_readwrite("route", &PlannerResult::route)
    .def_readwrite("cost", &PlannerResult::cost)
    .def_readwrite("max_range", &PlannerResult::max_range)
    .def_readwrite("speed", &PlannerResult::speed)
    .def_property_readonly("durations", &PlannerResult::durations)
    // TODO: I haven't figured out how to get the bindings for operator<< to
    // work yet, so for now, I'm stuck effectively re-writing them.
    .def("__str__", &to_string)
    .def("__repr__", [](const PlannerResult& self) {
      std::ostringstream os;
      os << "PlannerResult(route: '" << to_string(self) << "', ";
      os << "cost: " << self.cost << ", ";
      os << "max_range: " << self.max_range << " km, ";
      os << "speed: " << self.speed << " km/hr)";
      return os.str();
    })
  ;
}