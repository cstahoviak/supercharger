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
  };

  extern std::array<Charger, 303> NETWORK;

  std::ostream& operator<<(std::ostream& stream, const Charger& charger);
}   
/*************************************************/
/********* DO NOT MODIFY THIS FILE ************/
/*************************************************/
