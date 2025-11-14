#include "boink/world.h"

namespace boink
{
  World::World()
  {
    
  }

  void World::start(double dt)
  {
    bolid_manager.SetupSystems(dt);
  }
  void World::update(double dt)
  {
    bolid_manager.UpdateSystems(dt);
  }
}
