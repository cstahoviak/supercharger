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
      // NOTE: Must define a ctor to make use of "emplace_back"-like functions.
      Node(Charger charger) : charger_(std::move(charger)) {};

      // The length of time charging at this node (hrs).
      double duration{0};
      // The range of the vehicle upon arriving at this node.
      double arrival_range{0};
      // The post-charging range of the vehicle when departing this node.
      double departure_range{0};
      
      // "identity-oriented" getters
      const Charger& charger() const { return charger_; }
      const std::string& name() const { return charger_.name; }

      /**
       * @brief Resets the Node's attributes to their original values.
       */
      void Reset() { ResetNode(); }

    protected:
      /**
       * @brief A derived Node class may implement its own ResetNode() function.
       */
      virtual void ResetNode();

    private:
      // Store the charger associated with this node.
      Charger charger_;
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
   * @brief Dijkstra's Node adds a parent which it uses to traverse a linked
   * list of parents from the destination node back to the origin.
   * 
   * See the following StackOverflow page on info related to implementing a
   * graph node using a weak_ptr to store the parent node:
   * 
   * https://stackoverflow.com/questions/61723200/returning-a-weak-ptr-member-variable
   */
  class DijkstrasNode : 
    public Node, public std::enable_shared_from_this<DijkstrasNode>
  {
    public:
      // True if the node has been visited.
      bool visited{false};

      // The cost from this Node to the route origin.
      double cost{std::numeric_limits<double>::max()};

      // "value-oriented" getter and setter
      std::weak_ptr<DijkstrasNode> parent() const { return parent_; }
      void parent(std::shared_ptr<DijkstrasNode> parent) { parent_ = std::move(parent); }

    protected:
      void ResetNode() override;

    private:
      // Store the previous node on the route.
      std::weak_ptr<DijkstrasNode> parent_;
  };
} // end namespace supercharger