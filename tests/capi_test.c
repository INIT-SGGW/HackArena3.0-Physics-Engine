#include "boink/boink_c_api.h"

#include <stdio.h>

void printVehicleState(const BoinkVehicleState* state);
int main()
{
#ifdef WIN32
  const char* vehicle_filename=
    "C:\\Users\\igoru\\Source\\Repos\\HackArena3.0-Physics-Engine\\Bolid_F1.glb";
  const char* track_filename = 
    "C:\\Users\\igoru\\Source\\Repos\\HackArena3.0-Physics-Engine\\Bolid_Tor_test.glb";
#else
  const char* vehicle_filename="Bolid_F1.glb";
  const char* track_filename = "lowpoly_track_1_test_5.glb";
#endif

  unsigned int major,minor,patch;
  boink_get_engine_version(&major,&minor,&patch);
  printf("Engine verision: %d.%d.%d\n",major,minor,patch);
  boink_get_c_api_version(&major,&minor,&patch);
  printf("C API verision: %d.%d.%d\n",major,minor,patch);

  bool debug_enable=true;
  int code;

  if((code=boink_init(debug_enable))!=BOINK_OK)
  {
    printf("boink_init() failed: code %d.\n",code);
    return -1;
  }

  BoinkVehicleMeshHandle mesh_handle;
  if((code=boink_create_vehicle_mesh(vehicle_filename,&mesh_handle))!=BOINK_OK)
  {
    printf("boink_create_vehicle_mesh() failed: code %d.\n",code);
    boink_terminate();
    return -1;
  }
  
  BoinkHandle handle=boink_create_race(track_filename);
  if(handle==NULL){
    printf("boink_create_create_race() failed.\n");

    boink_destroy_vehicle_mesh(mesh_handle);
    boink_terminate();
    return -1;
  }

  BoinkVehicleModel model;
  model.center_of_mass.x=0.;
  model.center_of_mass.y=-0.4;
  model.center_of_mass.z=0.;
  model.mass=800.;
  model.max_steer_angle=90;
  model.mesh=mesh_handle;
  model.suspension_rest_length=0.4;
  model.wheel_radius=0.36;

  uint64_t id0;
  if((code=boink_spawn_vehicle(handle,&model,&id0))!=BOINK_OK)
  {
    printf("boink_spawn_vehicle() failed: code %d.\n",code);
    goto clear;
  }

  BoinkVec3 track_pos;
  track_pos.x=5.;
  track_pos.y=0.;
  track_pos.z=0.;
  if((code=boink_set_track_position(handle,&track_pos))!=BOINK_OK)
  {
    printf("boink_set_track_position() failed: code %d.\n",code);
    goto clear;
  }
  
  BoinkVec3 vehicle_pos;
  vehicle_pos.x=0.;
  vehicle_pos.y=13.;
  vehicle_pos.z=7.;
  if((code=boink_set_vehicle_position(handle,id0,&vehicle_pos))!=BOINK_OK)
  {
    printf("boink_set_vehicle_position() failed: code %d.\n",code);
    goto clear;
  }
  //BoinkControls controls;
  //controls.brake=0.0;
  //controls.steer=0.0;
  //controls.throttle=1.;
  //
  //if((code=boink_set_controls(handle,id0,&controls))!=BOINK_OK)
  //{
  //  printf("boink_set_controls() failed: code %d.\n",code);
  //  goto clear;
  //}
  Real prev=boink_get_time_debug();
  while(!boink_should_close_debug())
  {
    Real now=boink_get_time_debug();
    Real dt=now-prev;
    prev=now;

    if((code=boink_step_race(handle,dt))!=BOINK_OK)
    {
      printf("boink_step_race() failed: code %d.\n",code);
      goto clear;
    }
    boink_update_debug();
  }

  code=0;
  
clear:
  boink_destroy_vehicle_mesh(mesh_handle);
  boink_destroy_race(handle);
  boink_terminate();
  return code;
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
  printStateQuat(state->vehicle_orientation);
}
