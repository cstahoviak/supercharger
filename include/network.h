/*************************************************/
/********* DO NOT MODIFY THIS FILE ************/
/*************************************************/
#pragma once

#include <array>
#include <ostream>
#include <string>


namespace supercharger
{
  struct Charger
  {
    std::string name;
    double lat;
    double lon;
    double rate;

    // Default constructor required 
    // Charger() : name(""), lat(0), lon(0), rate(0) {};

    // Charger(std::string name, double lat, double lon, double rate) :
    //   name(name), lat(lat), lon(lon), rate(rate) {};

    // // Require == overload to use the Charger in std::unordered_set
    // inline bool operator==(const Charger& other) const {
    //   return name == other.name;
    // }

    // // Require hashable type to use the Charger in std::unordered_set
    // size_t operator()(const Charger& charger) const {
    //   return std::hash<std::string>{}(charger.name);
    // }
  };

  extern std::array<Charger, 303> network;

  std::ostream& operator<<(std::ostream& stream, const Charger& charger);
}   
/*************************************************/
/********* DO NOT MODIFY THIS FILE ************/
/*************************************************/
