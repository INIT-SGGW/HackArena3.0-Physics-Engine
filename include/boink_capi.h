#pragma once

#include "boink_export.h"

typedef void* BoinkHandle;

extern "C"{
  BOINK_EXPORT BoinkHandle create_engine();
  BOINK_EXPORT void destroy_engine(BoinkHandle handle);
}
