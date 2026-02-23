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
    :gui_(std::make_shared<WeatherGui>())
  {
    this->setTemperatureCelcius(temperature_celsius);
    this->setCloudiness(cloundiness);
    this->setRainIndensity(rain_indensity);

    gui_->cloudiness=&cloudiness_;
    gui_->rain_indensity=&rain_indensity_;
    gui_->temperature_celsius=&temperature_celsius_;
    gui_->wetness=&wetness_;
  }

  Weather::Weather(const Weather& other)
    : cloudiness_(other.cloudiness_),
      temperature_celsius_(other.temperature_celsius_),
      rain_indensity_(other.rain_indensity_),
      gui_(std::make_shared<WeatherGui>())
  {
    gui_->cloudiness=&cloudiness_;
    gui_->rain_indensity=&rain_indensity_;
    gui_->temperature_celsius=&temperature_celsius_;
    gui_->wetness=&wetness_;
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

    btScalar rain_add=rain_indensity_.getCurrent();

    btScalar sun_indensity=1.f-cloudiness_.getCurrent();
    btScalar sun_factor=
      sun_indensity*sun_indensity;
                                           
    btScalar temp_rate=(temperature_celsius_.getCurrent()-10.f)/30.f;
    btClamp(temp_rate,0.f,1.f);
    
    btScalar dry_rate=
      s_kTempRateConstant*temp_rate+
      s_kSunFactorConstant*sun_factor;

    btScalar wetness_factor=
      s_kRainAddConstant*rain_add-
      s_kDryRateConstant*dry_rate;

    wetness_+=wetness_factor/s_kTimeConstant;
    btClamp(wetness_,0.f,1.f);
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

  btScalar Weather::s_kSunFactorConstant=1.0f;
  btScalar Weather::s_kTempRateConstant=1.f;
  btScalar Weather::s_kDryRateConstant=0.05f;
  btScalar Weather::s_kRainAddConstant=0.3f;

  btScalar Weather::s_kTimeConstant=100.f;
}
