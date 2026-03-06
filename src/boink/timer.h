#pragma once

#include <LinearMath/btScalar.h>

namespace boink
{
  class Timer
  {
  public:
    Timer()
      :Timer(0.0f)
    {}

    Timer(btScalar target_duration)
      :elapsed_(0.f),target_(target_duration)
    {}

    void update(btScalar dt)
    {
      elapsed_+=dt;
    }

    void reset()
    {
      elapsed_=0.f;
    }

    void reset(btScalar target_duration)
    {
      this->reset();
      target_=target_duration;
    }

    bool hasFinised() const
    {
      return elapsed_>target_;
    }

    btScalar getCurrent() const
    {
      return elapsed_;
    }
  private:
    btScalar elapsed_;
    btScalar target_;
  };
}
