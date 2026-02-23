#pragma once

#include <LinearMath/btScalar.h>
#include <piksel/gui_object.hh>

#include "boink/simulators/simulator.h"
#include "boink/gui/weather_gui.h"

namespace boink
{
  class Weather : public Simulator
  {
  public:
    struct SmoothedValue
    {
    public:
      btScalar current;
      btScalar target;

      btScalar rate;
    public:
      SmoothedValue();
      SmoothedValue(btScalar current, btScalar target, btScalar rate=0.1);

      void update(btScalar dt);
    };
  public:
    Weather(
        btScalar cloundiness, 
        btScalar temperature, 
        btScalar rain_indensity);
    Weather(const Weather& other);
    Weather& operator=(const Weather&)=delete;

    void update(btScalar dt) override;
    void updateRender(Renderer* p_renderer) override;
    std::shared_ptr<piksel::GuiObject> getGui() override;

    const SmoothedValue& getCloudiness() const {return cloudiness_;}
    const SmoothedValue& getTemperatureCelsius() const {return temperature_celsius_;}
    const SmoothedValue& getRainIndensity() const {return rain_indensity_;}

    void setCloudiness(btScalar target, bool instant=false);
    void setTemperature(btScalar target, bool instant=false);
    void setRainIndensity(btScalar target, bool instant=false);
  public:
    static const Weather Sunny;
    static const Weather Rainy;
  private:
    SmoothedValue cloudiness_;
    SmoothedValue temperature_celsius_;
    SmoothedValue rain_indensity_;

    std::shared_ptr<WeatherGui> gui_;
  };
}
