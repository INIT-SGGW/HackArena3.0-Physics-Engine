#include "boink/smoothed_value.h"

namespace boink
{
  SmoothedValue::SmoothedValue()
    :SmoothedValue(0,0,0.f)
  {}

  SmoothedValue::SmoothedValue(
      btScalar current, btScalar target,btScalar transition_time)
    :current_(current)
  {
    this->setTarget(target,transition_time);
  }

  void SmoothedValue::update(btScalar dt)
  {
    current_+=rate_*dt;

    if((target_-current_)*rate_<0.)
      current_=target_;
  }

  void SmoothedValue::setTarget(
      btScalar target,btScalar transition_time)
  {
    target_=target;
    if(transition_time<0.01f)
    {
      current_=target;
      rate_=0.f;
    }
    else
      rate_=(target_-current_)/transition_time;
  }
}
