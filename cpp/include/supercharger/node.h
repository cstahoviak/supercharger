#pragma once
/**
 * @file node.h
 * @author Carl Stahoviak
 * @brief The Node struct represents a single node along the route.
 * @version 0.1
 * @date 2024-08-13
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/network.h"

#include <limits>
#include <memory>
#include <vector>


namespace supercharger
{
  /**
   * @brief A simple represenation of a single node in the network.
   */
  class Node
  {
    public:
      Node(Charger charger) : charger_(std::move(charger)) {};

      double duration{0};         // [hrs] charging time at this node 
      double arrival_range{0};    // [km] Vehicle range upon arrival at this node.
      double departure_range{0};  // [km] Post-charging range of vehicle.
      
      // "identity-oriented" getters
      const Charger& charger() const { return charger_; }
      const std::string& name() const { return charger_.name; }

      /**
       * @brief Resets the Node's attributes to their original values.
       */
      void Reset() { ResetNode_(); }

    protected:
      /**
       * @brief A derived Node class may implement its own ResetNode() function.
       */
      virtual void ResetNode_();

    private:
      // Store the charger associated with this node.
      Charger charger_;
  };


  /**
   * @brief Dijkstra's Node adds a parent which it uses to traverse a linked
   * list of parents from the destination node back to the origin.
   * 
   * See the following Stackoverflow page for info related to implementing a
   * graph node using a std::weak_ptr to store the parent node:
   * 
   * https://stackoverflow.com/questions/61723200/returning-a-weak-ptr-member-variable
   */
  class DijkstrasNode : 
    public Node, public std::enable_shared_from_this<DijkstrasNode>
  {
    public:
      DijkstrasNode(Charger charger) : Node(charger) {}

      // True if the node has been visited.
      bool visited{false};

      // The cost from this Node to the route origin.
      double cost{std::numeric_limits<double>::max()};

      // "value-oriented" getter and setter
      std::weak_ptr<DijkstrasNode> parent() const { return parent_; }
      void parent(std::shared_ptr<DijkstrasNode> parent) { 
        parent_ = std::move(parent);
      }

    protected:
      void ResetNode_() override;

    private:
      // Store the previous node on the route.
      std::weak_ptr<DijkstrasNode> parent_;
  };


  // Overload the string stream operator to output a Node.
  std::ostream& operator<<(std::ostream&, const Node&);


  /**
   * @brief Formats the final route output to comply with the format expected
   * by the provided "checker" executables.
   */
  std::ostream& operator<<(std::ostream&, const std::vector<Node>&);
  std::ostream& operator<<(std::ostream&, const std::vector<std::shared_ptr<Node>>&);

  /**
   * @brief Computes the charge time at the current node required to make it to
   * the next node.
   * 
   * @param current The current Node.
   * @param next The next node.
   * @return double The charge time required to make it to the next node.
   * Assumes that the arrival range at the 'next' node will be zero.
   */
  double GetChargeTime(const Node&, const Node&);
  double GetChargeTime(
    const std::shared_ptr<const Node>&,
    const std::shared_ptr<const Node>&
  );

  /**
   * @brief Computes the arrival range at the 'next' node after departing the
   * 'current' node.
   * 
   * @param current The current node.
   * @param next The next node.
   * @return double The arrival range at the 'next' node.
   */
  double GetArrivalRange(const Node&, const Node&);
  double GetArrivalRange(
    const std::shared_ptr<const Node>&,
    const std::shared_ptr<const Node>&
  );

  /**
   * @brief Computes the departure range at the current node given the current
   * node's arrival range and charging duration.
   * 
   * @param current The current node.
   * @return double The departure range after charging at the current node.
   */
  double GetDepartureRange(const Node&);
  double GetDepartureRange(const std::shared_ptr<const Node>&);
} // end namespace supercharger