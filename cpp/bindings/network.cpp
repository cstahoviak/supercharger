/**
 * @file network.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for...
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "network.h"

#include <pybind11/pybind11.h>

#include <sstream>

namespace py = pybind11;
using namespace supercharger;

void initNetwork(py::module_& m)
{
  py::class_<Charger>(m, "Charger")
    .def_readonly("name", &Charger::name)
    .def_readonly("lat", &Charger::lat)
    .def_readonly("lon", &Charger::lon)
    .def_readonly("rate", &Charger::rate)
    .def("__str__", [](const  Charger& self) { 
      std::ostringstream os;
      os << self;
      return os.str();
    })
    ;

  // m.def("__str__", py::overload_cast<std::ostream&, const Charger&>(&operator<<));
}