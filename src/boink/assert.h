#pragma once

#include "boink/logger.h"
#include <LinearMath/btScalar.h>

#define BOINK_ASSERT(expr,...)\
  do{\
    if(!(expr)){\
      BOINK_CRITICAL("Assertion failed: "#expr" " __VA_ARGS__);\
      btAssert(expr);\
    }\
  }while(false);
