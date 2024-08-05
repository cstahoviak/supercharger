#pragma once
/**
 * @file planner.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "network.h"

#include <chrono>
#include <optional>
#include <ostream>
#include <unordered_map>
#include <vector>


namespace supercharger
{
  /**
   * @brief Represents a single stop along the route.
   * 
   */
  struct Stop {
    // Must define a constructor to make use of "emplace_back" like functions
    Stop(Charger* charger, double duration, double range, Stop* parent) :
      charger(charger), duration(duration), range(range), parent(parent) {};

    // Store the current stop
    Charger* charger;
    // The length of time charging (hrs)
    double duration{0};
    // The current range or the vehicle after arriving at the stop
    double range{0};
    // Store the previous stop on the route
    Stop* parent;
  };

  class RoutePlanner
  {
    public:
      RoutePlanner(std::string&, std::string&);

      std::vector<std::optional<Stop>> PlanRoute();

      // Getters
      const std::unordered_map<std::string, Charger>& network() const {
        return network_;
      } 

    private:
      Charger* origin_;
      Charger* destination_;

      // Store some constants
      const double max_range_{320};   // [km]
      const double speed_{105};       // [km/hr]

      // Store the network
      std::unordered_map<std::string, Charger> network_;

      // Store each stop along the final route
      std::vector<std::optional<Stop>> route_;

      // The route-planning algorithms
      std::optional<Stop> BruteForce_(Stop&);
      void Dijkstra_();
      void AStar_();
  };

// Overload the string stream operator to output the route
std::ostream& operator<<(
  std::ostream& stream, const std::vector<std::optional<Stop>>& route);

} // end namespace supercharger
