#include "boink/simulators/weather.h"

#include <LinearMath/btMinMax.h>
#include <LinearMath/btScalar.h>
#include <piksel/gui_object.hh>

#include "boink/gui/weather_gui.h"

#include <memory>

namespace boink
{
  Weather::Weather(
      btScalar cloundiness, 
      btScalar temperature_celsius, 
      btScalar rain_indensity)
    :gui_(std::make_shared<WeatherGui>(this))
  {
    this->setTemperatureCelcius(temperature_celsius);
    this->setCloudiness(cloundiness);
    this->setRainIndensity(rain_indensity);
  }

  Weather::Weather(const Weather& other)
    : cloudiness_(other.cloudiness_),
      temperature_celsius_(other.temperature_celsius_),
      rain_indensity_(other.rain_indensity_),
      gui_(std::make_shared<WeatherGui>(this))
  {
  }

  void Weather::setCloudiness(btScalar target, btScalar transition_time)
  {
    cloudiness_.setTarget(target,transition_time);
  }

  void Weather::setTemperatureCelcius(btScalar target,btScalar transition_time)
  {
    temperature_celsius_.setTarget(target,transition_time);
  }

  void Weather::setRainIndensity(btScalar target,btScalar transition_time)
  {
    rain_indensity_.setTarget(target,transition_time);
  }

  void Weather::update(btScalar dt)
  {
    cloudiness_.update(dt);
    rain_indensity_.update(dt);
    temperature_celsius_.update(dt);

    btScalar rain_add=rain_indensity_.getCurrent()*(1.f-wetness_);

    btScalar sun_indensity=1.f-cloudiness_.getCurrent();
    btScalar sun_factor=
      sun_indensity*wetness_;
                                           
    btScalar temp_rate=temperature_celsius_.getCurrent()/30.f;
    btClamp(temp_rate,0.f,1.f);
    temp_rate*=wetness_;
    
    btScalar clound_rate=cloudiness_.getCurrent()*(1.f-wetness_);

    btScalar wetness_factor=
      s_kRainAddConstant*rain_add+
      -s_kTempRateConstant*temp_rate+
      -s_kSunFactorConstant*sun_factor+
      s_kCloundinessRateConstant*clound_rate;

    wetness_+=wetness_factor*dt*s_kWetnessSpeedConstant;
    //wetness_+=wetness_factor/s_kTimeConstant;
    //btClamp(wetness_,0.f,1.f);
  }

  void Weather::updateRender(Renderer* p_renderer)
  {
    if(!p_renderer)
      return;
  }
  
  std::shared_ptr<piksel::GuiObject> Weather::getGui()
  {
    return gui_;
  }

  const Weather Weather::Sunny{0,20,0};
  const Weather Weather::Rainy{1,20,0.5};
  const Weather Weather::HeavyRainy{1,20,1.0};

  btScalar Weather::s_kSunFactorConstant=0.02f;
  btScalar Weather::s_kTempRateConstant=0.06f;
  btScalar Weather::s_kCloundinessRateConstant=0.04f;
  btScalar Weather::s_kRainAddConstant=0.7f;
  btScalar Weather::s_kWetnessSpeedConstant=0.15f;
}
