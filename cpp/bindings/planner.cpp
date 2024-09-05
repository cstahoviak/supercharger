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
// Required for automatic type conversions between np.ndarray and std::vector
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace supercharger;

void initRoutePlanner(py::module_& m)
{
  // string stream operator
  // m.def("__str__", py::overload_cast)
}