#include <pybind11/pybind11.h>

namespace py = pybind11;

// Forward-declare class-specific binding functions
void initRoutePlanner(py::module_& m);

PYBIND11_MODULE(pysupercharger, m) {
  m.doc() = "Python bindings for the supercharger library.";

  initRoutePlanner(m);
}