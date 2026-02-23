#include "boink/simulators/weather.h"

#include <piksel/gui_object.hh>

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
}
