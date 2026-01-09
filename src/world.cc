#include "boink/world.h"
#include "boink/components/car_model.h"
#include "boink/simulation.h"

namespace boink
{
  World::World(CarModel&& car_model)
    :simulation(""),car_manager_(std::move(car_model))
  {}

  void World::setDebuger(DebugRender* dbg)
  {
    simulation.registerDebugDrawer(dbg);
  }

  void World::start(double dt)
  {
    car_manager_.SetupSystems(dt);
    time_passed_+=dt;
  }
  void World::update(double dt)
  {
    car_manager_.UpdateSystems(dt);
    simulation.step(dt);
    time_passed_+=dt;
  }

  CarManager<World::CarComponents,World::CarSystems>& World::getCarManager()
  {
    return car_manager_;
  }

  void World::addGround(const btVector3& dims, const btVector3& pos)
  {
    simulation.addGround(dims,pos);
  }
  void World::addSphere(btScalar radius, const btVector3& pos)
  {
    simulation.addSphere(radius,pos);
  }
}
