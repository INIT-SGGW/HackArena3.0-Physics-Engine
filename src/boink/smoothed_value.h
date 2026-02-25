#pragma once

#include <LinearMath/btScalar.h>

namespace boink
{
  struct SmoothedValue
  {
  public:
    SmoothedValue();
    SmoothedValue(btScalar current, btScalar target, btScalar trans_time=0.0f);

    void update(btScalar dt);
    void setTarget(btScalar target,btScalar transition_time);

    btScalar getCurrent() const { return current_;}
    btScalar getTarget() const { return target_;}
  private:
    btScalar current_;
    btScalar target_;
    btScalar rate_;
  };
}
