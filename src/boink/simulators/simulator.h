#pragma once

#include <LinearMath/btScalar.h>

#include <piksel/gui_object.hh>

#include "boink/debugger/renderer.h"

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
    virtual void updateRender(Renderer* p_renderer)=0;
    virtual std::shared_ptr<piksel::GuiObject> getGui()=0;
  protected:
    Simulator()=default;
  };
}
