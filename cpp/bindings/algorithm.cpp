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
#include "algorithm/dijkstras.h"
#include "algorithm/naive.h"

#include <pybind11/pybind11.h>
// NOTE: When you include any of the optional casters (pybind11/eigen.h, 
// pybind11/stl.h, etc.), you need to include them consistently in every 
// translation unit.
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace supercharger;

using AlgoType = PlanningAlgorithm::AlgorithmType;
using CostFcnType = PlanningAlgorithm::CostFunctionType;

void initPlanningAlgorithm(py::module_& m)
{
  py::class_<PlannerResult>(m, "PlannerResult")
    .def_readwrite("route", &PlannerResult::route)
    .def_readwrite("cost", &PlannerResult::cost)
    .def_readwrite("max_range", &PlannerResult::max_range)
    .def_readwrite("speed", &PlannerResult::speed)
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
  
  // Note that as-implmented the PlanningAlgorithm class is not extensible on
  // the python side. See the "Overriding virtual functions in Python" section
  // of the pybind11 docs for more info on how to use a "trampoline" class to
  // define dervied classes on the python side.
  py::class_<PlanningAlgorithm>(m, "PlanningAlgorithm")
    // .def(py::init<RoutePlanner*>(), py::arg("rp"))
    // .def_static("get_planning_algorithm", &PlanningAlgorithm::GetPlanningAlgorithm,
    //   py::arg("rp"), py::arg("algo_type"), py::arg("cost_type"))
    .def("plan_route", &PlanningAlgorithm::PlanRoute, 
      py::arg("origin"), py::arg("destination"))
    .def("ComputeCost", &PlanningAlgorithm::ComputeCost,
      py::arg("current"), py::arg("candidate"))
    // NOTE: Cannot bind protected or private members.
    // .def("_construct_final_route", 
    //   [](const PlanningAlgorithm& self, const Node* const final) {
    //     return self.ConstructFinalRoute_(final);
    //   }
    // )
  ;

  // TODO: I'm not sure how to deal with raw pointers as constructor arguments.
  // Pybind11 provides supports for smart pointers, so the solution here may be
  // to move away from using raw pointers entirely in the codebase in favor of
  // smart pointers.
  // py::class_<Naive, PlanningAlgorithm>(m, "Naive")
  //   .def(py::init<RoutePlanner*, CostFcnType>(), py::arg("rp"), py::arg("type"));
  // ;
    
}