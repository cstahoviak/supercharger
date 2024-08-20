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

// Define a compiler varaible based macro that will effectively hide all of our
// logging code if that variable is not set.
#ifdef DEBUG_INFO
  # define LOG(x) std::cout << x << std::endl;
#else
  #define LOG(x)
#endif