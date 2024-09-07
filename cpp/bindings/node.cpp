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
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace supercharger;

void initNode(py::module_& m)
{
  py::class_<Node>(m, "Node")
    .def(py::init<Charger*>(), py::arg("charger"))
    .def_readwrite("charger", &Node::charger)
    .def_readwrite("duration", &Node::duration)
    .def_readwrite("arrival_range", &Node::arrival_range)
    .def_readwrite("departure_range", &Node::departure_range)
    .def_readwrite("visited", &Node::visited)
    .def_readwrite("cost", &Node::cost)
    .def_readwrite("parent", &Node::parent)
    .def_property_readonly("name", &Node::name)
  ;
    
  m.def("__repr__", py::overload_cast<std::ostream&, const Node&>(&operator<<));
  // m.def("__repr__", py::overload_cast<std::ostream&, const Node* const>(&operator<<));
  ;
}