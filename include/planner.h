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
#include "logging.h"
#include "network.h"

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

    // Store the current charger
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
      enum class CostType {
        MINIMIZE_DIST_REMAINING,
        MINIMIZE_TIME_REMAINING
      };
    
    public:
      RoutePlanner();

      std::vector<Stop> PlanRoute(const std::string&, const std::string&, CostType);

      // Getters
      const std::unordered_map<std::string, Charger*>& network() const {
        return network_;
      } 

    private:
      // TODO: Switch to unique_ptr?
      Charger* origin_;
      Charger* destination_;

      // Store some constants
      const double max_range_{320};   // [km]
      const double speed_{105};       // [km/hr]

      // Store "brute force" const function weight parameters (weight remaining
      // travel time more heavily than charge time) NOTE: These weights were
      // arrived at after a bit of tuning, but more work could be done here to
      // further refine these values.
      const double weight_time_to_destination_ = 0.75;
      const double weight_time_to_charge_ = 0.25;

      // Store the network
      std::unordered_map<std::string, Charger*> network_;

      std::vector<Stop> InitializeRoute_(const std::string&, const std::string&);

      // Route-planning algorithms
      void BruteForce_(std::vector<Stop>&, CostType) const;
      void Dijkstra_(std::vector<Stop>&) const;
      void AStar_(std::vector<Stop>&) const;

      // Cost functions (args are const pointers to const Chargers)
      double ComputeChargeTime_(const Charger* const, const Charger* const) const;
      double ComputeCost_(const Charger* const, const Charger* const, CostType) const;
  };

// Overload the string stream operator to output the route
std::ostream& operator<<(std::ostream& stream, const std::vector<Stop>& route);

} // end namespace supercharger
