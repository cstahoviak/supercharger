#pragma once
/**
 * @file math.h
 * @author Carl Stahoviak
 * @brief Math utility functions.
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright (c) 2024
 */
#include "supercharger/node.h"

#include <cmath>

#define M_PI 3.14159265358979323846
#define M_EARTH_RADIUS_KM 6356.752       // [km]

namespace supercharger::math
{
  /**
   * @brief Uses the "Haversine" formula to compute the "great circle" distance
   * between two coordinates, (lat1, lon1) and (lat2, lon2).
   * 
   * https://www.movable-type.co.uk/scripts/latlong.html
   * 
   * NOTE: If the definition of a non-template function exists in a header, that
   * function must be marked "inline" to prevent "multiple definition" errors.
   * If the function is not marked "inline", the compiler will generate machine
   * code for that function in ~each~ translation unit that includes the header
   * where the function is defined, thus creating "multiple definition" errors.
   * 
   * @return double 
   */
  inline double great_circle_distance(
    const double lat1, const double lon1, const double lat2, const double lon2)
  {
    // Convert lat/long to radians
    const double phi1 = lat1 * M_PI / 180;
    const double phi2 = lat2 * M_PI / 180;
    const double delta_phi = (lat2 - lat1) * M_PI / 180;
    const double delta_lambda = (lon2 - lon1) * M_PI / 180;

    const double a =
      std::sin(delta_phi / 2) * std::sin(delta_phi / 2) +
      std::cos(phi1) * std::cos(phi2) * 
      std::sin(delta_lambda / 2) * std::sin(delta_lambda / 2);

    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // Return value in km
    return M_EARTH_RADIUS_KM * c;
  }

  /**
   * @brief Effectively a wrapper around the great_circle_distance() function
   * for use with the Charger type.
   * 
   * TODO: Would it make more sense for this to be in math.h?
   * 
   * @param charger1
   * @param charger2
   * @return double 
   */
  inline double distance(const Charger& charger1, const Charger& charger2)
  {
    return great_circle_distance(
      charger1.lat, charger1.lon, charger2.lat, charger2.lon);
  }

  /**
   * @brief Effectively a wrapper around the great_circle_distance() function
   * for use with the Node type.
   * 
   * @param node1 A reference a const Node.
   * @param node2 A reference to a const Node.
   * @return double 
   */
  inline double distance(const Node& node1, const Node& node2)
  {
    return distance(node1.charger(), node2.charger());
  }


} //end namespace supercharger