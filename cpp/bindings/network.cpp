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

namespace py = pybind11;
using namespace supercharger;

void initNetwork(py::module_& m)
{
  py::class_<Charger>(m, "Charger")
    .def_readwrite("name", &Charger::name)
    .def_readwrite("lat", &Charger::lat)
    .def_readwrite("lon", &Charger::lon)
    .def_readwrite("rate", &Charger::rate)
    ;

  m.def("__str__", py::overload_cast<std::ostream&, const Charger&>(&operator<<));
}