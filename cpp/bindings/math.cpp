/**
 * @file math.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for math functions.
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "supercharger/math/math.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace supercharger;

void initMath(py::module_& m)
{
  m.def("great_circle_distance", &math::great_circle_distance,
    py::arg("lat1"), py::arg("lon1"), py::arg("lat2"), py::arg("lon2"));
  
  m.def("distance",
    py::overload_cast<const Charger&, const Charger&>(&math::distance),
    py::arg("charger1"), py::arg("charger2"));
  m.def("distance",
    py::overload_cast<const Node&, const Node&>(&math::distance),
    py::arg("node1"), py::arg("node2"));

}