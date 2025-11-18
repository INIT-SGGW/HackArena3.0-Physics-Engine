#include "boink/boink_capi.h"

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

void test_heap_overflow_c() {
    // Allocate 10 integers (40 bytes)
    int* data = (int*)malloc(10 * sizeof(int));

    if (data == NULL) {
        perror("malloc failed");
        return;
    }

    printf("Running C ASan test: Heap Buffer Overflow...\n");

    // GOOD ACCESS: Accessing index 5 is safe
    data[5] = 100;

    // BAD ACCESS: Accessing index 10 (the 11th element), which is past the end.
    data[10] = 200; // <--- ASan should crash here!

    free(data);
}

void printCarState(struct BoinkCarState* out, double time);
int test();
int main()
{
  return test();
}

int test()
{
  BoinkCarModel car_model;
  BoinkVec3 vec;
  vec.x=-1.0;
  vec.y=0.0;
  vec.z=2.0;
  car_model.front_left_wheel=vec;
  vec.x=1.0;
  vec.y=0.0;
  vec.z=2.0;
  car_model.front_right_wheel=vec;
  vec.x=-1.0;
  vec.y=0.0;
  vec.z=-2.0;
  car_model.rear_left_wheel=vec;
  vec.x=1.0;
  vec.y=0.0;
  vec.z=-2.0;
  car_model.rear_right_wheel=vec;
  car_model.max_steer_angle=30;

  BoinkHandle handle=boink_create_world(&car_model);
  if(handle==NULL){
    fprintf(stderr,"Failed to create an engine\n");
    return -1;
  }

  uint64_t car_id;
  if(boink_spawn_car(handle,&car_id)!=BOINK_OK)
  {
    fprintf(stderr,"Failed to spawn a car\n");
    return -1;
  }

  double time=0.0;
  double dt=0.5;
  if(boink_begin_world(handle,time)!=BOINK_OK)
  {
    fprintf(stderr,"Failed to begin world\n");
    return -1;
  }

  struct BoinkControls controls;
  controls.brake=0.0;
  controls.steer=0.0;
  controls.throttle=1.0;

  if(boink_set_controls(handle,car_id,&controls)!=BOINK_OK)
  {
    fprintf(stderr,"Failed to set controls\n");
    return -1;
  }

  for(int i=0;i<2;i++)
  {
    time+=dt;
    if(boink_step(handle,dt)!=BOINK_OK)
    {
      fprintf(stderr,"Failed to begin world\n");
      return -1;
    }

    struct BoinkCarState out;
    if(boink_read_car_state(handle,car_id,&out)!=BOINK_OK)
    {
      fprintf(stderr,"Failed read car state\n");
      return -1;
    }
    printCarState(&out,time);
  }

  boink_destroy_world(handle);

  return 0;
}

void printCarState(struct BoinkCarState* out, double time)
{
#define VAR_PRINT_D(x) printf("  [%s]: %f\n",#x,x)
#define VAR_PRINT_U(x) printf("  [%s]: %" PRIu64 "\n",#x,x)
#define VAR_PRINT_I(x) printf("  [%s]: %d\n",#x,x)
    printf("Car state after t=%f\n",time);
    VAR_PRINT_D(out->brake_applied);
    VAR_PRINT_U(out->car_id);
    VAR_PRINT_D(out->engine_rpm);
    VAR_PRINT_I(out->gear);
    VAR_PRINT_D(out->orientation.roll);
    VAR_PRINT_D(out->orientation.pitch);
    VAR_PRINT_D(out->orientation.yaw);
    VAR_PRINT_D(out->position.x);
    VAR_PRINT_D(out->position.y);
    VAR_PRINT_D(out->position.z);
    VAR_PRINT_D(out->speed);
    VAR_PRINT_D(out->throttle_applied);
    VAR_PRINT_D(out->wheel_angles[0]);
    VAR_PRINT_D(out->wheel_angles[1]);
    VAR_PRINT_D(out->wheel_speeds[0]);
    VAR_PRINT_D(out->wheel_speeds[1]);
    VAR_PRINT_D(out->wheel_speeds[2]);
    VAR_PRINT_D(out->wheel_speeds[3]);
}
