#include "boink/world.h"

namespace boink
{
  World::World(const CarModel& car_model)
    :car_manager(car_model)
  {}

  void World::start(double dt)
  {
    car_manager.SetupSystems(dt);
    time_passed_+=dt;
  }
  void World::update(double dt)
  {
    car_manager.UpdateSystems(dt);
    time_passed_+=dt;
  }
}
