#pragma once
/**
 * @file brute_force.h
 * @author Carl Stahoviak
 * @brief My "naive" path planner.
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/algorithm/planner.h"


namespace supercharger::algorithm
{
  /**
   * @brief My "naive" path planner.
   */
  class NaivePlanner : public Planner
  {
    public:
      NaivePlanner(NaiveCostType type) : type_(type) {};

      /**
       * @brief The "naive" route planner.
       * 
       * TODO: Consider using std::unordered_set to store the "reachable"
       * (chargers that are within the current range of the vehicle post-
       * charging) and "closer to" (chargers that are closer to the destination
       * station than the current charger) sets of candidate chargers. Then, the
       * actual set of candidate chargers will be the intersection
       * (std::set_intersection) of the "reachable" and "closer to" sets. I
       * believe this will require defining both operator== and a hash function
       * for the supercharger::Charger class.
       * 
       * @param origin The origin node.
       * @param destination The destination node.
       * @param max_range [km] The vehicle's max range.
       * @param speed [km/hr] The vehicle's constant velocity.
       * @return PlannerResult The planner result.
       */
      PlannerResult PlanRoute(
        const std::string&,
        const std::string&,
        double,
        double) override;
        
      /**
       * @brief The "naive" planning algorithm cost function.
       * 
       * @param current The current node.
       * @param candidate The candidate "next" node.
       * @param speed The vehicle's constant velocity.
       * @return double The cost to reach the candidate node from the current
       * node.
       */
      double ComputeCost(const Node&, const Node&, double) const override;

    protected:
      /**
       * @brief Constructs the final route.
       * 
       * @return std::vector<Node> 
       */
      std::vector<Node> ConstructFinalRoute_(const Node&) override;

    private:
      Charger destination_;
      NaiveCostType type_;

      // Store the planned route.
      std::vector<std::shared_ptr<Node>> route_;

      // Store the "naive" cost function weight parameters (weight remaining
      // travel time more heavily than charge time) NOTE: These weights were
      // arrived at after a bit of tuning, but more work could be done here to
      // further refine these values.
      const double weight_time_to_destination_ = 0.75;
      const double weight_time_to_charge_ = 0.25;

      // Store the total cost (time in hrs) of the route.
      double total_cost_{0};

      /**
       * @brief The "naive" route planner is implemented via a recursive method.
       * 
       * @param origin The origin node.
       * @param destination The destination node.
       * @param max_range [km] The vehicle's max range.
       * @param speed [km/hr] The vehicle's constant velocity.
       */
      void PlanRouteRecursive_(
        const std::string&,
        const std::string&,
        double,
        double);
      
      /**
       * @brief Updates the route cost up to and including the charging time at
       * the most recently added node to the route.
       * 
       * @param speed [km/hr] The vehicle's constant speed.
       */
      void UpdateRouteCost_(double);
  };
} // end namespace supercharger