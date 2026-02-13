#pragma once

#include "boink/simulation/simulation.h"
#include "boink/simulators/vehicle/vehicle.h"

#include <string_view>
#include <unordered_map>
#include <memory>

namespace boink
{
  class Race : public Simulation
  {
  public:
    Race(std::string_view track_filename);

    std::shared_ptr<Track> getTrack();

    Simulator::ID addVehicle(const Vehicle::CreationInfo& ci);
    void removeVehicle(Simulator::ID id);
    std::shared_ptr<Vehicle> getVehicle(Simulator::ID id);
  private:
    std::shared_ptr<Track> track_;
    std::unordered_map<Simulator::ID,std::shared_ptr<Vehicle>> vehicles_;
  };
}
