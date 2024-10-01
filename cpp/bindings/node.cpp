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

#include <memory>
#include <sstream>

namespace py = pybind11;
using namespace supercharger;

void initNode(py::module_& m)
{
  py::class_<Node, std::shared_ptr<Node>>(m, "Node")
    .def(py::init<Charger>(), py::arg("charger"))
    .def("reset", &Node::Reset)
    .def_property_readonly("charger", &Node::charger)
    .def_readwrite("duration", &Node::duration)
    .def_readwrite("arrival_range", &Node::arrival_range)
    .def_readwrite("departure_range", &Node::departure_range)
    .def_readwrite("visited", &Node::visited)
    .def_readwrite("cost", &Node::cost)
    // Require a lambda to deal with getters and setters of the same name.
    .def_property("parent",
      [](const Node& self) { return self.parent(); },
      [](Node& self, std::shared_ptr<Node> parent) { self.parent(parent); }
    )
    .def_property_readonly("name", &Node::name)
    .def("__str__", [](const Node& self) { 
      std::ostringstream os;
      os << self;
      return os.str();
    })
    .def("__repr__", [](const Node& self) {
      std::string parent_name{"NULL"};
      if ( std::shared_ptr<Node> parent = self.parent().lock() ) {
        parent_name = parent->name();
      }

      std::ostringstream os;
      os << "Node(name: '" << self.name() << "', ";
      os << "rate: " << self.charger().rate << ", ";
      os << "duration: " << self.duration << ", ";
      os << "arrival_range: " << self.arrival_range << ", ";
      os << "departure_range: " << self.departure_range << ", ";
      os << "cost: " << self.cost << ", ";
      os << "visited: " << (self.visited) ? "True" : "False";
      os << ", parent: '" << parent_name << "')";
      return os.str();
    })
  ;
    
  // TODO: It might be cleaner if could bind __str__ and __repr__ to operator<<,
  // but I haven't been able to make that work yet.
  // m.def("__repr__", py::overload_cast<std::ostream&, const Node&>(&operator<<));
  // m.def("__repr__", py::overload_cast<std::ostream&, const Node* const>(&operator<<));
  ;
}