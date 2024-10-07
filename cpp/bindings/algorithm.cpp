/**
 * @file algorithm.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for the PlanningAlgorithm (and 
 *  PlanningAlgorithm-derived) classes.
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/algorithm.h"
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/algorithm/naive.h"

// NOTE: Must include the RoutePlanner header otherwise an "invalid use of
// incomplete type" error will occur when binding the PlanningAlgorithm ctor.
#include "supercharger/planner.h"

#include <pybind11/pybind11.h>
// NOTE: required because PlannerResult::route is a std::vector.
// NOTE: When you include any of the optional casters (pybind11/eigen.h, 
// pybind11/stl.h, etc.), you need to include them consistently in every 
// translation unit.
#include <pybind11/stl.h>

#include <sstream>

namespace supercharger::algorithm
{
  /**
   * @brief The PyPlanningAlgorithm "trampline" class allows the Optimizer class
   * to be extensible on the python side.
   */
  class PyPlanningAlgorithm : PlanningAlgorithm
  {
    public:
      // Inherit the constructor(s)
      using PlanningAlgorithm::PlanningAlgorithm;

      // "Trampoline" function(s) (required for each virtual function)
      PlannerResult PlanRoute(const std::string&, const std::string&) override;
      double ComputeCost(const Node&, const Node&) const override;

    protected:
      // TODO: I might not be able to bind proctected (and private) members??
      std::vector<Node> ConstructFinalRoute_(const Node&) override;
  };

  PlannerResult PyPlanningAlgorithm::PlanRoute(
    const std::string& origin, const std::string& destination)
  {
    PYBIND11_OVERRIDE_PURE(
      PlannerResult,      // Return type
      PlanningAlgorithm,  // Parent class
      PlanRoute,          // Name of C++ function (must match python name)
      origin,             // Argument(s)
      destination
    );
  }

  double PyPlanningAlgorithm::ComputeCost(
    const Node& current, const Node& neighbor) const
  {
    PYBIND11_OVERRIDE_PURE(
      double,             // Return type
      PlanningAlgorithm,  // Parent class
      ComputeCost,        // Name of C++ function (must match python name)
      current,            // Arguments(s)
      neighbor
    );
  }

  std::vector<Node> PyPlanningAlgorithm::ConstructFinalRoute_(
    const Node& final)
  {
    PYBIND11_OVERRIDE_PURE(
      std::vector<Node>,    // Return type
      PlanningAlgorithm,    // Parent class
      ConstructFinalRoute_, // Name of C++ function (must match python name)
      final                 // Arguments(s)
    );
  }
} // end namespace supercharger::algorithm

namespace py = pybind11;
using namespace supercharger;
using namespace supercharger::algorithm;

using AlgoType = PlanningAlgorithm::AlgorithmType;
using CostFcnType = PlanningAlgorithm::CostFunctionType;

std::string to_string(const PlannerResult& result) {
  std::ostringstream os;
  size_t idx = 0;
  for ( const Node& node : result.route ) {
    os << node;
    if ( idx < result.route.size() - 1 ) {
        os << ", ";
      }
      idx++;
  }
  return os.str();
}

void initPlanningAlgorithm(py::module_& m)
{
  py::class_<PlannerResult>(m, "PlannerResult")
    .def(py::init<>())
    .def(py::init<std::vector<Node>, double, double, double>(),
      py::arg("route"), py::arg("cost"), py::arg("max_range"), py::arg("speed"))
    .def_readwrite("route", &PlannerResult::route)
    .def_readwrite("cost", &PlannerResult::cost)
    .def_readwrite("max_range", &PlannerResult::max_range)
    .def_readwrite("speed", &PlannerResult::speed)
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

  py::enum_<AlgoType>(m, "AlgorithmType")
    .value("ASTAR", AlgoType::ASTAR)
    .value("DIJKSTRAS", AlgoType::DIJKSTRAS)
    .value("NAIVE", AlgoType::NAIVE)
    .export_values()
  ;

  py::enum_<CostFcnType>(m, "CostFunctionType")
    .value("MINIMIZE_DIST_TO_NEXT", CostFcnType::MINIMIZE_DIST_TO_NEXT)
    .value("MINIMIZE_DIST_REMAINING", CostFcnType::MINIMIZE_DIST_REMAINING)
    .value("MINIMIZE_TIME_REMAINING", CostFcnType::MINIMIZE_TIME_REMAINING)
    .value("NONE", CostFcnType::NONE)
    .export_values()
  ;
  
  py::class_<PlanningAlgorithm, PyPlanningAlgorithm>(m, "PlanningAlgorithm")
    .def(py::init<RoutePlanner*>(), py::arg("rp"))
    .def_static("get_planning_algorithm", &PlanningAlgorithm::GetPlanningAlgorithm,
      py::arg("rp"), py::arg("algo_type"), py::arg("cost_type"))
    .def("plan_route", &PlanningAlgorithm::PlanRoute, 
      py::arg("origin"), py::arg("destination"))
    .def("compute_cost", &PlanningAlgorithm::ComputeCost,
      py::arg("current"), py::arg("neighbor"))
    .def("reset", &PlanningAlgorithm::Reset)
    // NOTE: Cannot bind protected or private members.
    // .def("_construct_final_route", &PlanningAlgorithm::ConstructFinalRoute_)
  ;

  py::class_<Naive, PlanningAlgorithm>(m, "NaivePlanner")
    .def(py::init<RoutePlanner*, CostFcnType>(),
      py::arg("rp"), py::arg("cost_type"));
  ;

    py::class_<Dijkstras, PlanningAlgorithm>(m, "Dijkstras")
    .def(py::init<RoutePlanner*>(), py::arg("rp"));
  ;   
}