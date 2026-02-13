#include "boink/simulation/race.h"

#include "boink/exception.h"
#include "boink/simulators/track/track.h"

#include <sstream>

namespace boink
{
  Race::Race(std::string_view track_filename)
  {
    track_=std::make_shared<Track>(track_filename,this->getDynamicsWorld());
    this->getSimulators().add(track_);
  }

  std::shared_ptr<Track> Race::getTrack()
  {
    return track_;
  }

  Simulator::ID Race::addVehicle(const Vehicle::CreationInfo& ci)
  {
    auto vehicle=
        std::make_shared<Vehicle>(ci,track_,this->getDynamicsWorld());
    Simulator::ID vehicle_id=this->getSimulators().add(vehicle);

    vehicles_.emplace(vehicle_id,vehicle);

    return vehicle_id;
  }

  void Race::removeVehicle(Simulator::ID id)
  {
    auto it=vehicles_.find(id);
    if(it==vehicles_.end())
    {
      std::stringstream ss;
      ss<<"Vehicle with id="<<id<<"was not found.";
      throw Exception(
          Exception::Type::NotFoundError,
          ss.str());
    }

    bool is_deleted=this->getSimulators().remove(id);

    if(!is_deleted)
      throw Exception(
          Exception::Type::InternalError,
          "Containers misalignment. Panic.");

    assert(is_deleted && it!=vehicles_.end());
    assert(!is_deleted && it==vehicles_.end());

    if(it!=vehicles_.end())
      vehicles_.erase(it);
  }

  std::shared_ptr<Vehicle> Race::getVehicle(Simulator::ID id)
  {
    auto it=vehicles_.find(id);

    if(it==vehicles_.end())
    {
      std::stringstream ss;
      ss<<"Vehicle with id="<<id<<"was not found.";
      throw Exception(
          Exception::Type::NotFoundError,
          ss.str());
    }

    return vehicles_.at(id);
  }
}
