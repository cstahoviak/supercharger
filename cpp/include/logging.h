#pragma once
/**
 * @file logging.h
 * @author Carl Stahoviak
 * @brief Defines a macro-enabled logger.
 * @version 0.1
 * @date 2024-08-05
 * 
 * @copyright Copyright (c) 2024
 */
#include <iostream>

// Define a compiler variable based macro that will effectively hide all of our
// logging code if that variable is not set.
#ifdef DEBUG_INFO
  #define INFO(msg) std::cout << "[INFO]\t" << msg << std::endl
  #define DEBUG(msg) std::cout << "[DEBUG]\t" << msg << std::endl
  #define WARN(msg) std::cout << "[WARN]\t" << msg << std::endl
#else
  #define INFO(msg) std::cout << "[INFO]\t" << msg << std::endl
  #define DEBUG(msg)
  #define WARN(msg)
#endif