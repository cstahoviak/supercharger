/**
 * @file naive.cpp
 * @author Carl Stahoviak
 * @brief The "naive" optimizer definition.
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/optimize/naive.h"
#include "supercharger/logging.h"

namespace supercharger::optimize
{
  PlannerResult NaiveOptimizer::Optimize(const PlannerResult& result) const {
    const std::vector<Node>& route = result.route;
    if ( route.size() < 4 ) {
      INFO("The route contains 3 nodes or fewer and cannot be optimized.");
      return result;
    }
    
    // Find the node with the fastest charger in the route (that's not the
    // first, last or second to last node in the route).
    double max_rate{0};
    size_t argmax{0};
    for ( size_t idx = 1; idx < route.size() - 2; idx++) {
      if ( route.at(idx).charger().rate > max_rate ) {
        max_rate = route.at(idx).charger().rate;
        argmax = idx;
      }
    }

    // Initialize the optimized route as a copy of the provided route.
    std::vector<Node> optimized = route;
    Node& max_node = optimized.at(argmax);

    DEBUG("Charger at " << max_node.name() << " with a rate of " <<
      max_node.charger().rate << " km/hr is the fastest charger on the route.");

    // The charge time at the "max-rate" node will be a function of the
    // distance remaining to the destination.
    double dist_remaining{0};
    for ( size_t idx = argmax; idx < optimized.size() - 1; idx++ ) {
      const Node& current = optimized.at(idx);
      const Node& next = optimized.at(idx + 1);
      dist_remaining += math::distance(current, next);
    }
    DEBUG("Distance remaining between " << max_node.name() << " and " << 
      route.back().name() << ": " << dist_remaining << " km.");

    // If the distance remaining is less than the max range of the vehicle, only
    // charge the vehicle enough at the max-rate node to make it to the
    // destination. Otherwise, the distance remaining is greater than the 
    // maximum range of the vehicle, so we can charge the vehicle fully. 
    max_node.departure_range = ( dist_remaining <= result.max_range ) ? 
      dist_remaining - max_node.departure_range : result.max_range;

    // Compute the charging duration at the max-rate node.
    max_node.duration = (max_node.departure_range - max_node.arrival_range) / 
      max_node.charger().rate;

    // For the max-rate node, update the newly calculated charge duration.
    DEBUG("Charging for " << max_node.duration << " hrs at " <<
      max_node.name() << ".");
    DEBUG("Leaving " << max_node.name() << " with a range of " <<
      max_node.departure_range << " km.");

    // Update the optimized route from the max-rate node to the end.
    for ( size_t idx = argmax + 1; idx < optimized.size(); idx++ ) {
      const Node& prev = optimized.at(idx - 1);
      Node& current = optimized.at(idx);

      // For all remaining nodes on the route, update the range remaining
      // after arriving at the node.
      current.arrival_range =
        prev.departure_range - math::distance(prev, current);

      // Update the current node's cost.
      current.cost = prev.cost + prev.duration + 
        math::distance(prev, current) / result.speed;

      if ( idx < optimized.size() - 1 ) {
        // For all nodes not the destination, update the charge duration if
        // the range remaining is less than the distance to the next node.
        const Node& next = optimized.at(idx + 1);
        double dist_to_next = math::distance(current, next);
        if ( current.arrival_range < dist_to_next ) {
          // Charge just long enough to make it to the next node.
          current.duration =
            (dist_to_next - current.arrival_range) / current.charger().rate;
        }
        else {
          current.duration = 0;
        }

        DEBUG("Updating charge duration at " << current.name() <<
          " from " << route.at(idx).duration << " hrs to " <<
          current.duration << " hrs.");

        // Update the departure range.
        current.departure_range = current.arrival_range +
          current.duration * current.charger().rate;
      }
    }
    
    return { optimized, optimized.back().cost, result.max_range, result.speed };
  }
} // end namespace supercharger::optimize