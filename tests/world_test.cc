#include "boink/world.h"

using namespace boink;
using namespace Eigen;

int main()
{
  CarModel car_model;
  car_model.front_left_wheel=Vector3d(-1.0,0.0,2.0);
  car_model.front_right_wheel=Vector3d(1.0,0.0,2.0);
  car_model.rear_left_wheel=Vector3d(-1.0,0.0,-2.0);
  car_model.rear_right_wheel=Vector3d(1.0,0.0,-2.0);
  car_model.max_steer_angle_deg=30;

  World world(car_model);
  world.car_manager.addCar();

  world.start(0.0);
  world.update(10.0);
}
