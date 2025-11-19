#include <stdio.h>

#include "boink/boink_capi.h"

int main() {
  BoinkCarModel car_model;
  BoinkVec3 vec;
  vec.x = -1.0;
  vec.y = 0.0;
  vec.z = 2.0;
  car_model.front_left_wheel = vec;
  vec.x = 1.0;
  vec.y = 0.0;
  vec.z = 2.0;
  car_model.front_right_wheel = vec;
  vec.x = -1.0;
  vec.y = 0.0;
  vec.z = -2.0;
  car_model.rear_left_wheel = vec;
  vec.x = 1.0;
  vec.y = 0.0;
  vec.z = -2.0;
  car_model.rear_right_wheel = vec;
  car_model.max_steer_angle = 30;

  BoinkHandle handle = boink_create_world(&car_model);
  if (handle == NULL) {
    fprintf(stderr, "Failed to create an engine\n");
    return -1;
  }

  uint64_t car_id;
  if (boink_spawn_car(handle, &car_id) != BOINK_OK) {
    fprintf(stderr, "Failed to spawn a car\n");
    return -1;
  }

  if (boink_begin_world(handle, 0.0) != BOINK_OK) {
    fprintf(stderr, "Failed to begin world\n");
    return -1;
  }

  struct BoinkControls controls;
  controls.brake = 0.0;
  controls.steer = 0.0;
  controls.throttle = 1.0;

  if (boink_set_controls(handle, car_id, &controls) != BOINK_OK) {
    fprintf(stderr, "Failed to set controls\n");
    return -1;
  }

  for (int i = 0; i < 1; i++) {
    if (boink_step(handle, 0.5) != BOINK_OK) {
      fprintf(stderr, "Failed to begin world\n");
      return -1;
    }

    struct BoinkCarState out_state;
    if (boink_read_car_state(handle, car_id, &out_state) != BOINK_OK) {
      fprintf(stderr, "Failed read car state\n");
      return -1;
    }
  }

  boink_destroy_world(handle);

  return 0;
}
