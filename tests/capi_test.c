#include "boink/boink_c_api.h"

#include <stdio.h>

#define BOINK_ERR_PRINT(x) \
  {\
    int result=x;\
    if(result!=BOINK_OK){\
      printf("Failed running function %s with code %d\n",#x,result);\
      return -1;\
    }\
  }

void printVehicleState(const BoinkVehicleState* state);
int main()
{
  const char* vehicle_filename="C:\\Users\\igoru\\Source\\Repos\\HackArena3.0-Physics-Engine\\Bolid_F1.glb";
  const char* track_filename = "C:\\Users\\igoru\\Source\\Repos\\HackArena3.0-Physics-Engine\\Bolid_Tor_test.glb";

  BOINK_ERR_PRINT(boink_init(true));

  BoinkVehicleMeshHandle mesh_handle;
  BOINK_ERR_PRINT(boink_create_vehicle_mesh(vehicle_filename,&mesh_handle));
  
  BoinkHandle handle=boink_create_race(track_filename);
  if(handle==NULL){
    printf("Failed running function boink_create_race\n");
    return -1;
  }

  BoinkVehicleModel model;
  model.center_of_mass.x=0.;
  model.center_of_mass.y=1.;
  model.center_of_mass.z=0.;
  model.mass=800.;
  model.max_steer_angle=1.5;
  model.mesh=mesh_handle;
  model.suspension_rest_length=0.5;
  model.wheel_radius=0.36;

  uint64_t id0;
  BOINK_ERR_PRINT(boink_spawn_vehicle(handle,&model,&id0));
  
  Real prev=boink_get_time_debug();
  while(!boink_should_close_debug())
  {
    Real now=boink_get_time_debug();
    Real dt=now-prev;
    prev=now;

    BOINK_ERR_PRINT(boink_step_race(handle,dt));
    boink_update_debug();
  }
  
  boink_destroy_vehicle_mesh(mesh_handle);
  boink_destroy_race(handle);
  boink_terminate();
}

#define printStateReal(x)\
  printf(#x ": %f\n",x);

#define printStateInt(x)\
  printf(#x ": %d\n",x);

#define printStateVec3(v)\
  printf(#v ": {%f,%f,%f}\n",v.x,v.y,v.z);

#define printStateQuat(q)\
  printf(#q ": {%f,%f,%f,%f}\n",q.x,q.y,q.z,q.w);

void printVehicleState(const BoinkVehicleState* state)
{
  printStateReal(state->speed);
  printStateVec3(state->chassis_position);
  printStateQuat(state->chassis_orientation);
}
