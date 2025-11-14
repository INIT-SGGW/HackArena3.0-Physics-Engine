#include "boink/world.h"

using namespace boink;

int main()
{
  World world;
  world.bolid_manager.addBoild();

  world.start(0.0);
  world.update(10.0);
}
