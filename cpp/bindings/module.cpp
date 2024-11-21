#include <pybind11/pybind11.h>

namespace py = pybind11;

// Forward-declare class-specific binding functions
void initPlanningAlgorithm(py::module_& m);
void initMath(py::module_& m);
void initNetwork(py::module_& m);
void initNode(py::module_& m);
void initOptimizer(py::module_& m);
void initSupercharger(py::module_& m);
void initTypes(py::module_& m);

PYBIND11_MODULE(pysupercharger, m) {
  m.doc() = "Python bindings for the supercharger library.";

  initPlanningAlgorithm(m);
  initMath(m);
  initNetwork(m);
  initNode(m);
  initOptimizer(m);
  initSupercharger(m);
  initTypes(m);
}