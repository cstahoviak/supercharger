/**
 * @file algorithm.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for the Planner (and Planner-derived) classes.
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"
#include "supercharger/algorithm/dijkstras.h"
#include "supercharger/algorithm/naive.h"

#include <pybind11/pybind11.h>
// Need to include the funcional header in any translation unit where 
// DijkstrasCostFcnType appears.
#include <pybind11/functional.h>
// NOTE: required because PlannerResult::route is a std::vector.
// NOTE: When you include any of the optional casters (pybind11/eigen.h, 
// pybind11/stl.h, etc.), you need to include them consistently in every 
// translation unit.
#include <pybind11/stl.h>

#include <sstream>


namespace supercharger::algorithm
{
  /**
   * @brief The PyPlanner "trampoline" class allows the Optimizer class
   * to be extensible on the python side.
   */
  class PyPlanner : Planner
  {
    public:
      // Inherit the constructor(s) (no ctor to inherit from in this case).

    protected:
      // NOTE: I might not be able to bind proctected (and private) members?

      // "Trampoline" function(s) (required for each virtual function)
      PlannerResult PlanRoute_(
        const std::string&,
        const std::string&,
        double,
        double) override;
      
      double ComputeCost_(
        const Node&,
        const Node&,
        double,
        double) const override;

      
      std::vector<std::shared_ptr<Node>> ConstructRoute_(const Node&) override;
  };

  PlannerResult PyPlanner::PlanRoute_(
    const std::string& origin,
    const std::string& destination,
    double max_range,
    double speed)
  {
    PYBIND11_OVERRIDE_PURE(
      PlannerResult,      // Return type
      Planner,            // Parent class
      PlanRoute_,         // Name of C++ function (must match python name)
      origin,             // Argument(s)
      destination,
      max_range,
      speed
    );
  }

  double PyPlanner::ComputeCost_(
    const Node& current,
    const Node& neighbor,
    double max_range,
    double speed) const
  {
    PYBIND11_OVERRIDE_PURE(
      double,             // Return type
      Planner,            // Parent class
      ComputeCost_,       // Name of C++ function (must match python name)
      current,            // Arguments(s)
      neighbor,
      max_range,
      speed
    );
  }

  std::vector<std::shared_ptr<Node>> PyPlanner::ConstructRoute_(
    const Node& final)
  {
    PYBIND11_OVERRIDE_PURE(
      std::vector<std::shared_ptr<Node>>,    // Return type
      Planner,            // Parent class
      ConstructRoute_,    // Name of C++ function (must match python name)
      final               // Arguments(s)
    );
  }
} // end namespace supercharger::algorithm

namespace py = pybind11;
using namespace supercharger;
using namespace supercharger::algorithm;

using AlgoType = Planner::AlgorithmType;
using CostFcnType = Planner::CostFunctionType;

std::string to_string(const PlannerResult& result) {
  std::ostringstream os;
  size_t idx = 0;
  for ( const std::shared_ptr<const Node>& node : result.route ) {
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

  py::enum_<AlgoType>(m, "AlgorithmType")
    .value("ASTAR", AlgoType::ASTAR)
    .value("DIJKSTRAS", AlgoType::DIJKSTRAS)
    .value("NAIVE", AlgoType::NAIVE)
    .export_values()
  ;

  py::enum_<CostFcnType>(m, "CostFunctionType")
    .value("NAIVE_MINIMIZE_DIST_TO_NEXT", CostFcnType::NAIVE_MINIMIZE_DIST_TO_NEXT)
    .value("NAIVE_MINIMIZE_DIST_REMAINING", CostFcnType::NAIVE_MINIMIZE_DIST_REMAINING)
    .value("NAIVE_MINIMIZE_TIME_REMAINING", CostFcnType::NAIVE_MINIMIZE_TIME_REMAINING)
    .value("DIJKSTRAS_SIMPLE", CostFcnType::DIJKSTRAS_SIMPLE)
    .value("DIJKSTRAS_OPTIMIZED", CostFcnType::DIJKSTRAS_OPTIMIZED)
    .export_values()
  ;
  
  py::class_<Planner, PyPlanner>(m, "Planner")
    .def(py::init<>())
    .def_static("get_planner",
      py::overload_cast<CostFcnType>(&Planner::GetPlanner), py::arg("cost_type"))
    .def_static("get_planner",
      py::overload_cast<AlgoType, DijkstrasCostFcnType>(&Planner::GetPlanner),
      py::arg("algo_type"), py::arg("cost_fcn"))
    .def("plan", &Planner::Plan, 
      py::arg("origin"), py::arg("destination"), py::arg("max_range"), py::arg("speed"))

    // NOTE: Cannot bind protected or private members.
  ;

  py::class_<NaivePlanner, Planner>(m, "NaivePlanner")
    .def(py::init<CostFcnType>(), py::arg("cost_type"))
  ;

  py::class_<Dijkstras, Planner>(m, "DijkstrasPlanner")
    .def(py::init<CostFcnType>(), py::arg("cost_type"))
    .def(py::init<DijkstrasCostFcnType>(), py::arg("cost_f"))
    // TODO: Make the Dijkstras planner pickleable for multiprocessing. See
    // https://pybind11.readthedocs.io/en/stable/advanced/classes.html#pickling-support
  ;

  m.def("ConstructRoute", &algorithm::ConstructRoute, py::arg("final"));
  m.def("dijkstras_simple_cost", &algorithm::SimpleCost,
    py::arg("current"), py::arg("neighbor"), py::arg("max_range"), py::arg("speed"));
  m.def("dijkstras_optimized_cost", &algorithm::OptimizedCost,
    py::arg("current"), py::arg("neighbor"), py::arg("max_range"), py::arg("speed"));
}