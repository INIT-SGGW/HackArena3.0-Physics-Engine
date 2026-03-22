#pragma once

#include <LinearMath/btVector3.h>

namespace boink
{
  struct BoundingBox
  {
    btVector3 top_left;
    btVector3 top_right;
    btVector3 bottom_left;
    btVector3 bottom_right;
  };
}

