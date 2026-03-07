#pragma once

#include "boink/simulation/simulation.h"
#include "boink/simulators/vehicle/ghost_mode_settings.h"
#include "boink/simulators/vehicle/vehicle.h"
#include "boink/debugger/controller.h"
#include "boink/simulators/weather.h"

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
    int update(
        btScalar dt,
        int max_sub_steps=15,
        btScalar fixed_delta_time=1.f/120.f,
        btScalar max_delta_time=0.1) override;
    void updateDebug() override;

    std::shared_ptr<Track> getTrack();
    std::shared_ptr<Weather> getWeather();

    Simulator::ID addVehicle(const Vehicle::CreationInfo& ci);
    void removeVehicle(Simulator::ID id);
    std::shared_ptr<Vehicle> getVehicle(Simulator::ID id);
    auto& getVehicles() {return vehicles_;}

    void enableGhostMode(GhostModeSettings ghost_settings);
    void disableGhostMode();

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
    std::shared_ptr<Weather> weather_;
    std::shared_ptr<Track> track_;
    std::unordered_map<Simulator::ID,std::shared_ptr<Vehicle>> vehicles_;

    bool ghost_enabled_=false;
    GhostModeSettings ghost_settings_;

    void* user_ptr_=nullptr;
  };
}
