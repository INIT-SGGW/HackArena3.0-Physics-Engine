#include "boink/world.h"

using namespace boink;
using namespace Eigen;

void test_heap_overflow() {
    // Allocate an array of 10 integers (40 bytes) on the heap
    int* data = new int[10];

    // GOOD ACCESS: Accessing index 5 is safe
    data[5] = 100;

    // BAD ACCESS: Accessing index 10 (the 11th element), which is past the end of the allocated block.
    // This is a one-byte out-of-bounds access.
    data[10] = 200; // <--- ASan should crash here!

    delete[] data;
}

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

  test_heap_overflow();
}
