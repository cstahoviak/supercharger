/**
 * @file node.cpp
 * @author Carl Stahoviak
 * @brief Pybind11 bindings for the Node class.
 * @version 0.1
 * @date 2024-09-05
 * 
 * @copyright Copyright (c) 2024
 * 
 * Some notes on inheriting from std::enable_shared_from_this and what that
 * requires of the python bindings.
 * 
 * - Pybind11 uses holder types to manage the lifetime of C++ objects exposed to
 * Python. By default, it uses std::unique_ptr for this purpose.
 * - The following python error occurs if you have a holder-type mismatch
 * between a base class (Node) and a derived class (DijkstrasNode).
 *    "ImportError: generic_type: type "DijkstrasNode" has a non-default holder
 *    type while its base "supercharger::Node" does not."
 * - The error above occurs when you have a base class with a default holder
 * type (usually std::unique_ptr) and a derived class that requires a different
 * holder type (e.g., std::shared_ptr).
 * - Pybind11 doesn't allow for a mismatch in holder types between a base class
 * and its derived classes.
 * 
 * Because of this requirement, both Node and DijkstrasNode must inherit from
 * std::shared_ptr<node-type>.
 */
#include "supercharger/node.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <sstream>

namespace py = pybind11;
using namespace supercharger;

std::optional<std::shared_ptr<DijkstrasNode>> get_parent(const DijkstrasNode& node)
{
  if ( std::shared_ptr<DijkstrasNode> parent = node.parent().lock() ) {
    return parent;
  }
  else {
    return {};
  }
}


void initNode(py::module_& m)
{
  py::class_<Node, std::shared_ptr<Node>>(m, "Node")
    .def(py::init<Charger>(), py::arg("charger"))
    .def_readwrite("duration", &Node::duration)
    .def_readwrite("arrival_range", &Node::arrival_range)
    .def_property_readonly("departure_range", &Node::departure_range)
    .def_property_readonly("charger", &Node::charger)
    .def_property_readonly("name", &Node::name)
    .def("reset", &Node::Reset)
    
    .def("__str__", [](const Node& self) { 
      std::ostringstream os;
      os << self;
      return os.str();
    })
    .def("__repr__", [](const Node& self) {
      std::ostringstream os;
      os << "Node(name: '" << self.name() << "', ";
      os << "rate: " << self.charger().rate << ", ";
      os << "duration: " << self.duration << ", ";
      os << "arrival_range: " << self.arrival_range << ", ";
      os << "departure_range: " << self.departure_range() << ")";
      return os.str();
    })
  ;

  py::class_<DijkstrasNode, Node, std::shared_ptr<DijkstrasNode>>(m, "DijkstrasNode")
    .def(py::init<Charger>(), py::arg("charger"))
    .def_readwrite("visited", &DijkstrasNode::visited)
    .def_readwrite("cost", &DijkstrasNode::cost)
    // Require a lambda to deal with getters and setters of the same name.
    .def_property("parent",
      [](const DijkstrasNode& self) { return get_parent(self); },
      [](DijkstrasNode& self, std::shared_ptr<DijkstrasNode> parent) {
        self.parent(parent);
      }
    )
    .def("__str__", [](const DijkstrasNode& self) { 
      std::ostringstream os;
      os << self;
      return os.str();
    })
    .def("__repr__", [](const DijkstrasNode& self) {
      std::string parent_name{"NULL"};
      if ( std::shared_ptr<Node> parent = self.parent().lock() ) {
        parent_name = parent->name();
      }

      std::ostringstream os;
      os << "DijkstrasNode(name: '" << self.name() << "', ";
      os << "rate: " << self.charger().rate << ", ";
      os << "duration: " << self.duration << ", ";
      os << "arrival_range: " << self.arrival_range << ", ";
      os << "departure_range: " << self.departure_range() << ", ";
      os << "cost: " << self.cost << ", ";
      os << "visited: " << (self.visited) ? "True" : "False";
      os << ", parent: '" << parent_name << "')";
      return os.str();
    })
  ;

  m.def("get_charge_time",
    py::overload_cast<const Node&, const Node&>(&GetChargeTime),
    py::arg("current"), py::arg("next"));
  m.def("get_arrival_range",
    py::overload_cast<const Node&, const Node&>(&GetArrivalRange),
    py::arg("current"), py::arg("next"));
    
  // TODO: It might be cleaner if could bind __str__ and __repr__ to operator<<,
  // but I haven't been able to make that work yet.
  // m.def("__repr__", py::overload_cast<std::ostream&, const Node&>(&operator<<));
  // m.def("__repr__", py::overload_cast<std::ostream&, const Node* const>(&operator<<));
  ;
}