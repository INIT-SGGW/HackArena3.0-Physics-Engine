#include "boink/boink_capi.h"

#include <stdio.h>
#include <stdlib.h>

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

int main()
{
  BoinkCarModel car_model;
  BoinkVec3 vec;
  vec.x=-1.0;
  vec.y=0.0;
  vec.z=1.0;
  car_model.front_left_wheel=vec;
  vec.x=1.0;
  vec.y=0.0;
  vec.z=1.0;
  car_model.front_right_wheel=vec;
  vec.x=-1.0;
  vec.y=0.0;
  vec.z=-1.0;
  car_model.rear_left_wheel=vec;
  vec.x=1.0;
  vec.y=0.0;
  vec.z=-1.0;
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

  if(boink_begin_world(handle,0.0)!=BOINK_OK)
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
    if(boink_step(handle,0.5)!=BOINK_OK)
    {
      fprintf(stderr,"Failed to begin world\n");
      return -1;
    }

    struct BoinkCarState out_state;
    if(boink_read_car_state(handle,car_id,&out_state)!=BOINK_OK)
    {
      fprintf(stderr,"Failed read car state\n");
      return -1;
    }
  }

  test_heap_overflow_c();

  char* buff = malloc(10);
  boink_destroy_world(handle);
}
