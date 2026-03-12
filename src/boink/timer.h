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
      :elapsed_(0.f),target_(target_duration),has_finised_(false)
    {}

    void update(btScalar dt)
    {
      if(elapsed_>=target_)
      {
        has_finised_=true;
        elapsed_=target_;
      }
      else
      {
        has_finised_=false;
        elapsed_+=dt;
      }
    }

    void reset()
    {
      has_finised_=false;
      elapsed_=0.f;
    }

    void reset(btScalar target_duration)
    {
      this->reset();
      target_=target_duration;
    }

    bool hasFinised() const
    {
      return has_finised_;
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
      has_finised_=true;
    }
  private:
    btScalar elapsed_;
    btScalar target_;

    bool has_finised_;
  };
}
