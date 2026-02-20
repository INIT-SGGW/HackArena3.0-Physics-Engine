#include "boink/simulators/weather.h"

#include <piksel/gui_object.hh>

#include <memory>

namespace boink
{
  Weather::SmoothedValue::SmoothedValue()
    :current(0),target(0),rate(0.1)
  {}

  Weather::SmoothedValue::SmoothedValue(
      btScalar current, btScalar target, btScalar rate)
    :current(current),target(target),rate(rate)
  {}

  void Weather::SmoothedValue::update(btScalar dt)
  {
    btScalar diff=target-current;
    current+=diff*rate*dt;

    if(btFabs(diff)<0.01)
      current=target;
  }

  Weather::Weather(
      btScalar cloundiness, 
      btScalar temperature_celsius, 
      btScalar rain_indensity)
    :gui_(std::make_shared<WeatherGui>())
  {
    this->setTemperature(temperature_celsius,true);
    this->setCloudiness(cloundiness,true);
    this->setRainIndensity(rain_indensity,true);

    gui_->cloudiness=this->getCloudiness().current;
    gui_->rain_indensity=this->getRainIndensity().current;
    gui_->temperature_celsius=this->getTemperatureCelsius().current;

    gui_->cloudiness_target=this->getCloudiness().target;
    gui_->rain_indensity_target=this->getRainIndensity().target;
    gui_->temperature_celsius_target=this->getTemperatureCelsius().target;
  }

  void Weather::setCloudiness(btScalar target, bool instant)
  {
    if(instant)
      cloudiness_.current=target;
    cloudiness_.target=target;
  }

  void Weather::setTemperature(btScalar target, bool instant)
  {
    if(instant)
      temperature_celsius_.current=target;
    temperature_celsius_.target=target;
  }

  void Weather::setRainIndensity(btScalar target, bool instant)
  {
    if(instant)
      rain_indensity_.current=target;
    rain_indensity_.target=target;
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

    this->updateGui();
  }
  
  std::shared_ptr<piksel::GuiObject> Weather::getGui()
  {
    return gui_;
  }

  void Weather::updateGui()
  {
    setTemperature(gui_->temperature_celsius_target,false);
    setCloudiness(gui_->cloudiness_target,false);
    setRainIndensity(gui_->rain_indensity_target,false);

    gui_->cloudiness=this->getCloudiness().current;
    gui_->rain_indensity=this->getRainIndensity().current;
    gui_->temperature_celsius=this->getTemperatureCelsius().current;

    gui_->cloudiness_target=this->getCloudiness().target;
    gui_->rain_indensity_target=this->getRainIndensity().target;
    gui_->temperature_celsius_target=this->getTemperatureCelsius().target;
  }

  const Weather Weather::Sunny{0,20,0};
  const Weather Weather::Rainy{1,20,0.5};
}
