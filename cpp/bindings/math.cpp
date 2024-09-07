/**
 * @file math.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "math.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace supercharger;

void initMath(py::module_& m)
{
  m.def("great_circle_distance", &great_circle_distance,
    py::arg("lat1"), py::arg("lon1"), py::arg("lat2"), py::arg("lon2"));
  
  m.def("compute_distance",
    py::overload_cast<const Charger* const, const Charger* const>(&compute_distance));
  m.def("compute_distance",
    py::overload_cast<const Node* const, const Node* const>(&compute_distance));
  m.def("compute_distance",
    py::overload_cast<const Node&, const Node&>(&compute_distance));

}