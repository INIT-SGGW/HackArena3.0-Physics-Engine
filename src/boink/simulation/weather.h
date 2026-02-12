#pragma once

#include <LinearMath/btScalar.h>

namespace boink
{
  class Weather
  {
  public:
    struct SmoothedValue
    {
      btScalar current;
      btScalar target;

      btScalar rate;

      SmoothedValue(btScalar current, btScalar target, btScalar rate=0.1);
      void update(btScalar dt);
    };
  public:
    void update(btScalar dt);

    SmoothedValue getCloudiness() const {return cloudiness_;}
    SmoothedValue getTemperature() const {return temperature_;}
    SmoothedValue getRainIndensity() const {return rain_indensity_;}

    void setCloudiness(btScalar target) {cloudiness_.target=target;}
    void setTemperature(btScalar target) {temperature_.target=target;}
    void setRainIndensity(btScalar target) {rain_indensity_.target=target;}
  private:
    SmoothedValue cloudiness_;
    SmoothedValue temperature_;
    SmoothedValue rain_indensity_;
  };
}
