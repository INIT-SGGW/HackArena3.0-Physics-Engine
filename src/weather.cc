#include "boink/simulation/weather.h"

namespace boink
{
  Weather::SmoothedValue::SmoothedValue(
      btScalar current, btScalar target, btScalar rate)
    :current(current),target(target),rate(rate)
  {}

  void Weather::SmoothedValue::update(btScalar dt)
  {
    btScalar diff=target-current;
    current+=diff*rate*dt;

    if(btFabs(diff)<1e-5)
      current=target;
  }

  void Weather::update(btScalar dt)
  {
    cloudiness_.update(dt);
    rain_indensity_.update(dt);
    temperature_.update(dt);
  }
}
