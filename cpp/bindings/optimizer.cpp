/**
 * @file optimizer.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for...
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "optimizer/naive.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace supercharger;


void initOptimizer(py::module_& m)
{
  py::enum_<Optimizer::OptimizerType>(m, "OptimizerType")
    .value("NAIVE", Optimizer::OptimizerType::NAIVE)
    .export_values()
  ;

  py::class_<Optimizer>(m, "Optimizer")
    .def_static("get_optimizer", &Optimizer::GetOptimizer, py::arg("type"))
    .def("optimize", &Optimizer::Optimize, py::arg("result"))
  ;

  py::class_<NaiveOptimizer, Optimizer>(m, "NaiveOptimizer")
    .def(py::init<>())
  ;
}