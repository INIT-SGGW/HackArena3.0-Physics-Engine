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
      if(elapsed_>=target_)
        elapsed_=target_;
      else
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
      return target_==elapsed_;
    }

    bool isRunning() const
    {
      return elapsed_!=0.f && !hasFinised();
    }

    btScalar getCurrent() const
    {
      return elapsed_;
    }

    btScalar getTarget() const
    {
      return target_;
    }

    void setElapsedToFinish() 
    {
      elapsed_=target_;
    }
  private:
    btScalar elapsed_;
    btScalar target_;
  };
}
