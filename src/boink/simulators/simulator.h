#pragma once

#include <LinearMath/btScalar.h>

#include "boink/debugger/debugger.h"

#include <cstdint>

namespace boink
{
  class Simulator
  {
  public:
    typedef uint64_t ID;
  public:
    virtual ~Simulator() noexcept=default;

    virtual void update(btScalar dt) = 0;
    virtual void updateDebug(Debugger* p_dbg) {(void)p_dbg;}
  protected:
    Simulator()=default;
  };
}
