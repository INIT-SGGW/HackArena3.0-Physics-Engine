#include "boink/world.h"

namespace boink
{
  World::World()
  {
  }

  void World::update(double dt)
  {
    bolid_manager.UpdateSystems(dt);
  }
}
