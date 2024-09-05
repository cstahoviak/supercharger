/**
 * @file node.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "node.h"

#include <iomanip>

namespace supercharger
{
  std::ostream& operator<<(std::ostream& stream, const Node& node) {
    return stream << std::addressof(node);
  }

  std::ostream& operator<<(std::ostream& stream, const Node* const node) {
    stream << node->name();
    if ( node->duration > 0 ) {
      stream << ", " << std::setprecision(6) << node->duration;
    }
    return stream;
  }
}