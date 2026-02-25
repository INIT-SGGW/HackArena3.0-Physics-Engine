#pragma once

#include <LinearMath/btScalar.h>
#include <piksel/gui_object.hh>

#include "boink/simulators/simulator.h"
#include "boink/smoothed_value.h"

namespace boink
{
  class WeatherGui;
  class Weather : public Simulator
  {
  public:
    friend class WeatherGui;
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

    btScalar getWetness() const {return wetness_;}

    const SmoothedValue& getCloudiness() const {return cloudiness_;}
    const SmoothedValue& getTemperatureCelsius() const {return temperature_celsius_;}
    const SmoothedValue& getRainIndensity() const {return rain_indensity_;}

    void setCloudiness(btScalar target, btScalar transition_time=0.f);
    void setTemperatureCelcius(btScalar target, btScalar transition_time=0.f);
    void setRainIndensity(btScalar target, btScalar transition_time=0.f);
  public:
    static const Weather Sunny;
    static const Weather Rainy;
    static const Weather HeavyRainy;
  private:
    static btScalar s_kSunFactorConstant;
    static btScalar s_kTempRateConstant;
    static btScalar s_kCloundinessRateConstant;
    static btScalar s_kRainAddConstant;
    static btScalar s_kWetnessSpeedConstant;
  private:
    SmoothedValue cloudiness_;
    SmoothedValue temperature_celsius_;
    SmoothedValue rain_indensity_;

    btScalar wetness_=0.0f;

    std::shared_ptr<WeatherGui> gui_;
  };
}
