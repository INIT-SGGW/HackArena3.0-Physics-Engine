#include "boink/simulation/race.h"

#include "boink/exception.h"
#include "boink/simulators/track/track.h"
#include "boink/gui/race_gui.h"
#include "boink/debugger/vehicle_controller.h"

#include <sstream>

namespace boink
{
  Race::Race(
        btScalar gravity_acceleration,
        std::string_view track_filename,
        Debugger* p_dbg)
    :
      Simulation(gravity_acceleration,p_dbg,std::make_shared<RaceGui>()),
      weather_(std::make_shared<Weather>(Weather::Sunny)),
      track_(std::make_shared<Track>(
            track_filename,weather_,this->getDynamicsWorld()))
  {
    this->addSimulator(track_);
    this->addSimulator(weather_);

    this->getDynamicsWorld()->getBroadphase()->
      getOverlappingPairCache()->setOverlapFilterCallback(&filter);
  }

  Race::~Race()
  {
    if(p_dbg_)
      p_dbg_->setControllers({});
  }

  std::shared_ptr<Track> Race::getTrack()
  {
    return track_;
  }

  std::shared_ptr<Weather> Race::getWeather()
  {
    return weather_;
  }

  Simulator::ID Race::addVehicle(const Vehicle::CreationInfo& ci)
  {
    auto vehicle=
        std::make_shared<Vehicle>(ci,track_,this->getDynamicsWorld());
    Simulator::ID vehicle_id=this->addSimulator(vehicle);

    vehicles_.emplace(vehicle_id,vehicle);

    if(p_dbg_)
      p_dbg_->setControllers(this->getControllers());
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

    bool is_deleted=this->removeSimulator(id);

    if(!is_deleted)
      throw Exception(
          Exception::Type::InternalError,
          "Containers misalignment. Panic.");

    assert(is_deleted && it!=vehicles_.end());

    if(it!=vehicles_.end())
      vehicles_.erase(it);

    if(p_dbg_)
      p_dbg_->setControllers(this->getControllers());
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

  std::vector<std::pair<Simulator::ID,std::shared_ptr<Controller>>> 
    Race::getControllers() const 
  {
    std::vector<std::pair<Simulator::ID,std::shared_ptr<Controller>>> 
      controllers;
    controllers.reserve(vehicles_.size());

    for(const auto& p : vehicles_)
    {
      controllers.emplace_back(
          p.first,std::make_shared<VehicleController>(p.second));
    }

    return controllers;
  }

  int Race::update(
      btScalar dt,
      int max_sub_steps,
      btScalar fixed_delta_time,
      btScalar max_delta_time)
  {
    int steps=Simulation::update(dt,max_sub_steps,fixed_delta_time,max_delta_time);

    return steps;
  }

  void Race::updateDebug()
  {
    Simulation::updateDebug();

    this->updateGui();
  }

  void Race::updateGui()
  {
    if(!gui_)
      return;

#ifdef NDEBUG
    // We use faster alternative because it shuld be guranteed that
    // gui pointer is of type RaceGui
    RaceGui* p_race_gui=static_cast<RaceGui*>(gui_.get());
#else
    RaceGui* p_race_gui=dynamic_cast<RaceGui*>(gui_.get());
    assert(p_race_gui!=nullptr);
#endif
    p_race_gui->num_vehicles=vehicles_.size();
  }
}
