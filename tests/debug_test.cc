#include "boink/debug_drawer.h"
#include "boink/simulation.h"


using namespace boink;
using namespace piksel;

int main()
{
  DebugDrawer dbg;
  Simulation sim("Bolid_Tor_test.glb");

  sim.registerDebugDrawer(&dbg);

  float prev=dbg.getTime();
  while(dbg)
  {
    float now=dbg.getTime();
    float dt=now-prev;
    prev=now;


    dbg.drawFrameOrigin();
    sim.step(dt);
    dbg.update();
  }
  return 0;
}
