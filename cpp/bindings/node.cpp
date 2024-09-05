/**
 * @file node.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for...
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "node.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace supercharger;

void initNode(py::module_& m)
{
  py::class_<Node>(m, "Node")
    .def(py::init<Charger*>(), py::arg("charger"))
    .def_readonly("charger", &Node::charger)
    .def_readonly("duration", &Node::duration)
    .def_readonly("arrival_range", &Node::arrival_range)
    .def_readonly("departure_range", &Node::departure_range)
    .def_readonly("visited", &Node::visited)
    .def_readonly("cost", &Node::cost)
    .def_readonly("parent", &Node::parent)
    .def_property_readonly("name", &Node::name)
    ;

  m.def("__str__", 
    py::overload_cast<std::ostream&, const Node&>(&operator<<),
    py::arg("os"), py::arg("node"));
  m.def("__str__", 
    py::overload_cast<std::ostream&, const Node* const>(&operator<<),
    py::arg("os"), py::arg("node"));
}