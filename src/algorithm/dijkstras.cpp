/**
 * @file djikstras.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "algorithm/dijkstras.h"

namespace supercharger
{
  void Dijkstras::PlanRoute(std::vector<Stop>& route) {
    // 1. Mark all "nodes"/stops as unvisited

    // 2. Assign a "distance to start" value to each node in the network.
    // Initially, this value will be "infinity" since no path is yet known to
    // these "unvisited" nodes.

    // 3. From the unvisited set, select the current node to be the one with
    // the smallest known distance from the origin. If the current node is the
    // destination node, we're done; otherwise, continue to find the shortest
    // path to all reachable nodes.

    // 4. For the current node, consider all of its unvisited neighbors and
    // update their distance through the current node. Compare the newly
    // calculated distance to the one currently assigned to the neighbor, and
    // assign the neighbor the smaller distance.

    // 5. Once we've considered all the unvisited neighbors of the current node,
    // mark the current node as visited and remove it from the unvisited set.

    // 6. Return to step 3.

    // 7. Once the loop (steps 3-5) exits, every node will contain the shortest
    // distance from the start node.
    return;
  }
}