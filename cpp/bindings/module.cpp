#include <pybind11/pybind11.h>

namespace py = pybind11;

// Forward-declare class-specific binding functions
void initPlanningAlgorithm(py::module_& m);
void initNetwork(py::module_& m);
void initNode(py::module_& m);
void initRoutePlanner(py::module_& m);

PYBIND11_MODULE(pysupercharger, m) {
  m.doc() = "Python bindings for the supercharger library.";

  initPlanningAlgorithm(m);
  initNetwork(m);
  initNode(m);
  initRoutePlanner(m);
}