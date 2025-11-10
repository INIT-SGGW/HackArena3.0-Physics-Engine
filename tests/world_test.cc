#include "boink/world.h"

#include "boink/components/transform.h"
#include "boink/components/velocity.h"

using namespace boink;

int main()
{
  World world;
  world.bolid_manager.addBoild(
    Transform{
      Eigen::Vector3d{1.0,2.0,3.0},
      Eigen::Vector3d{1.0,2.0,3.0}
    },
    Velocity{
      Eigen::Vector3d{2.0,4.0,3.0}
    }
  );
  world.update(10.0);
  world.update(10.0);
}
