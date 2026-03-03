#pragma once

#include "boink/simulation/simulation.h"
#include "boink/simulators/vehicle/vehicle.h"
#include "boink/debugger/controller.h"
#include "boink/simulators/weather.h"
#include "boink/simulation/race_filter.h"

#include <string_view>
#include <unordered_map>
#include <memory>

namespace boink
{
  class Race : public Simulation
  {
  public:
    Race(
        btScalar gravity_acceleration,
        std::string_view track_filename,
        Debugger* p_dbg=nullptr);
    ~Race() ;
    void updateDebug() override;

    std::shared_ptr<Track> getTrack();
    std::shared_ptr<Weather> getWeather();

    Simulator::ID addVehicle(const Vehicle::CreationInfo& ci);
    void removeVehicle(Simulator::ID id);
    std::shared_ptr<Vehicle> getVehicle(Simulator::ID id);
    auto& getVehicles() {return vehicles_;}

    // Temporary soliton
    void setUserPtr(void* ptr)
    {user_ptr_=ptr;}
    void* getUserPtr() const
    {return user_ptr_;}
  private:
    std::vector<std::pair<Simulator::ID,std::shared_ptr<Controller>>> 
      getControllers() const;
    void updateGui();
  private:
    static void bulletCustomNearCallback(
        btBroadphasePair& pair,
        btCollisionDispatcher& dispatcher,
        const btDispatcherInfo& info);
  private:
    RaceFilter filter;
    std::shared_ptr<Weather> weather_;
    std::shared_ptr<Track> track_;
    std::unordered_map<Simulator::ID,std::shared_ptr<Vehicle>> vehicles_;

    void* user_ptr_=nullptr;
  };
}
