/**
 * @file algorithm.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for...
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/algorithm.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace supercharger;

using AlgoType = PlanningAlgorithm::AlgorithmType;
using CostFcnType = PlanningAlgorithm::CostFunctionType;

void initPlanningAlgorithm(py::module_& m)
{
  py::class_<PlannerResult>(m, "PlannerResult")
    .def_readonly("route", &PlannerResult::route)
    .def_readonly("cost", &PlannerResult::cost)
    .def_readonly("max_range", &PlannerResult::max_range)
    .def_readonly("speed", &PlannerResult::speed)
    ;

  py::enum_<AlgoType>(m, "AlgorithmType")
    .value("ASTAR", AlgoType::ASTAR)
    .value("DIJKSTRAS", AlgoType::DIJKSTRAS)
    .value("NAIVE", AlgoType::NAIVE)
    .export_values()
    ;

  py::enum_<CostFcnType>(m, "CostFunctionType")
    .value("ASTAR", CostFcnType::MINIMIZE_DIST_TO_NEXT)
    .value("DIJKSTRAS", CostFcnType::MINIMIZE_DIST_REMAINING)
    .value("NAIVE", CostFcnType::MINIMIZE_TIME_REMAINING)
    .value("NODE", CostFcnType::NONE)
    .export_values()
    ;
  
  // py::class_<PlanningAlgorithm>(m, "PlanningAlgorithm")
  //   // .def(py::init<RoutePlanner*>(), py::arg("rp"))
  //   .def(py::init<>())
  //   .def_static("get_planning_algorithm", &PlanningAlgorithm::GetPlanningAlgorithm,
  //     py::arg("rp"), py::arg("algo_type"), py::arg("cost_type"))
  //   ;
    
}