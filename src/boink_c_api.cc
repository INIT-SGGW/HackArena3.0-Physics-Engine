#include "boink/boink_c_api.h"

#include "boink/debug_drawer.h"
#include "boink/exception.h"
#include "boink/simulation.h"
#include "boink/simulation/vehicle.h"
#include "boink/simulation/vehicle_mesh.h"

#include "boink/simulation/wheel_position.h"
#include "boink/version.h"

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <exception>
#include <memory>
#include <iostream>
#include <numbers>

#define HANDLE_EXCEPTIONS(block)       \
try {                                  \
    block;                              \
} catch (const boink::Exception& e) {  \
    std::cout << e.what() << std::endl; \
    return exceptionType2api(e.getType()); \
} catch (const std::exception& e) {    \
    std::cout << "STL exception" << std::endl; \
    std::cout << e.what() << std::endl; \
    return BOINK_ERR_INTERNAL;          \
} catch (...) {                         \
    std::cout << "Unknown Exception" << std::endl; \
    return BOINK_ERR_INTERNAL;          \
}

static int exceptionType2api(boink::Exception::Type type);
static BoinkVec3 bt2boink(btVector3 bt_vec);
//static btVector3 boink2bt(BoinkVec3 boink_vec);
static BoinkQuaternion bt2boink(btQuaternion bt_quat);
//static btQuaternion boink2bt(BoinkQuaternion boink_quat);

static boink::DebugDrawer* g_drawer=NULL;

int boink_get_c_api_version(unsigned int *out_major,
                                 unsigned int *out_minor,
                                 unsigned int *out_patch)
{
  if (out_major == nullptr || 
    out_minor == nullptr || 
    out_patch == nullptr)
    return BOINK_ERR_INVALID_ARG;

  *out_major=BOINK_C_API_VERSION_MAJOR;
  *out_minor=BOINK_C_API_VERSION_MINOR;
  *out_patch=BOINK_C_API_VERSION_PATCH;

  return BOINK_OK;
}

int boink_get_engine_version(unsigned int *out_major,
                                 unsigned int *out_minor,
                                 unsigned int *out_patch)
{
  if (out_major == nullptr ||
    out_minor == nullptr ||
    out_patch == nullptr)
    return BOINK_ERR_INVALID_ARG;

  *out_major=BOINK_VERSION_MAJOR;
  *out_minor=BOINK_VERSION_MINOR;
  *out_patch=BOINK_VERSION_PATCH;

  return BOINK_OK;
}

int boink_init(bool debug_drawer_enable)
{
  if(debug_drawer_enable)
  {
    HANDLE_EXCEPTIONS(
      g_drawer=new boink::DebugDrawer())
  }
  return BOINK_OK;
}

void boink_terminate()
{
  if(g_drawer!=NULL)
    delete g_drawer;
}

BoinkHandle boink_create_race(const char* track_glb_filename)
{
  if(track_glb_filename==nullptr)
    return nullptr;
  try
  {
    boink::Simulation* p_sim=new boink::Simulation(track_glb_filename);
    if(g_drawer!=NULL)
      p_sim->registerDebugDrawer(g_drawer);
    return reinterpret_cast<BoinkHandle>(p_sim);
  }
  catch(boink::Exception& e)
  {
    std::cout << e.what() << std::endl;
    return nullptr;
  }
  catch (const std::exception& e) 
  {
    std::cout << "STL exception" << std::endl;
    std::cout << e.what() << std::endl;
    return nullptr;
  }
  catch(...)
  {
    std::cout << "Unknown Exception" << std::endl;
    return nullptr;
  }
}

int boink_create_vehicle_mesh(
    const char* glb_model_filename,
    BoinkVehicleMeshHandle* out_mesh_handle)
{
  if(glb_model_filename==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(out_mesh_handle==nullptr)
    return BOINK_ERR_INVALID_ARG;

  HANDLE_EXCEPTIONS(
    *out_mesh_handle=reinterpret_cast<BoinkVehicleMeshHandle>(
        new boink::VehicleMesh(glb_model_filename)))

  return BOINK_OK;
}

void boink_destroy_vehicle_mesh(BoinkVehicleMeshHandle handle)
{
  if(handle!=nullptr)
    delete reinterpret_cast<boink::VehicleMesh*>(handle);
}

int boink_get_race_duration(BoinkHandle handle, Real* out_dur)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(out_dur==nullptr)
    return BOINK_ERR_INVALID_ARG;

  *out_dur=p_sim->getSimulationDuration();
  return BOINK_OK;
}

int boink_step_race(BoinkHandle handle, Real dt)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(dt<0)
    return BOINK_ERR_INVALID_ARG;

  p_sim->step(dt);
  return BOINK_OK;
}

void boink_update_debug()
{
  if(g_drawer!=NULL)
  {
    g_drawer->drawFrameOrigin();
    g_drawer->update();
  }
}

Real boink_get_time_debug()
{
  if(g_drawer!=NULL)
    return (Real)g_drawer->getTime();

  return 0;
}

bool boink_should_close_debug()
{
  if(g_drawer!=NULL)
    return !static_cast<bool>(*g_drawer);

  return true;
}

void boink_destroy_race(BoinkHandle handle)
{
  if(handle!=nullptr)
    delete (boink::Simulation*) handle;
}

int boink_spawn_vehicle(
    BoinkHandle handle, 
    const BoinkVehicleModel* p_vehicle_model,
    uint64_t* out_vehicle_id)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(p_vehicle_model==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(out_vehicle_id==nullptr)
    return BOINK_ERR_INVALID_ARG;

  boink::Vehicle::CreationInfo create_info;
  create_info.center_of_mass.setX(p_vehicle_model->center_of_mass.x);
  create_info.center_of_mass.setY(p_vehicle_model->center_of_mass.y);
  create_info.center_of_mass.setZ(p_vehicle_model->center_of_mass.z);
  
  create_info.mass=p_vehicle_model->mass;
  create_info.wheel_radius=p_vehicle_model->wheel_radius;
  create_info.suspension_rest_length=p_vehicle_model->suspension_rest_length;
  create_info.max_steer_angle =
      p_vehicle_model->max_steer_angle * std::numbers::pi / 180.0;

  create_info.mesh=std::shared_ptr<const boink::VehicleMesh>(
      (const boink::VehicleMesh*)p_vehicle_model->mesh,
      [](const boink::VehicleMesh*){});

  // TODO
  // I think try is not needed here but it must be checked
  HANDLE_EXCEPTIONS(
    *out_vehicle_id=p_sim->addVehicle(create_info))

  return BOINK_OK;
}

int boink_despawn_vehicle(BoinkHandle handle, uint64_t vehicle_id)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;

  HANDLE_EXCEPTIONS(
    p_sim->removeVehicle(vehicle_id))
  
  return BOINK_OK;
}

int boink_set_controls(
    BoinkHandle handle, 
    uint64_t vehicle_id, 
    const BoinkControls* controls)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(controls==nullptr)
    return BOINK_ERR_INVALID_ARG;
  
  boink::Vehicle* p_vehicle=nullptr;
  HANDLE_EXCEPTIONS(
    p_vehicle=&p_sim->getVehicle(vehicle_id))

  if(controls->throttle>1. || controls->throttle<0.)
  {
    std::cout<<"BoinkControls::throttle should be in the range [0.0, 1.0]."<<std::endl;
    return BOINK_ERR_INVALID_ARG;
  }
    
  if(controls->brake>1. || controls->brake<0.)
  {
    std::cout<<"BoinkControls::brake should be in the range [0.0, 1.0]."<<std::endl;
    return BOINK_ERR_INVALID_ARG;
  }

  if(controls->steer>1. || controls->steer<-1.)
  {
    std::cout<<"BoinkControls::brake should be in the range [-1.0, 1.0]."<<std::endl;
    return BOINK_ERR_INVALID_ARG;
  }

  p_vehicle->setEngineForce(controls->throttle);
  p_vehicle->setBrake(controls->brake);

  Real steer=std::abs(controls->steer);
  boink::Vehicle::TurnDirection dir=
    controls->steer<0.0?
    boink::Vehicle::TurnDirection::Left:
    boink::Vehicle::TurnDirection::Right;
  p_vehicle->setSteering(steer,dir);

  return BOINK_OK;
}

int boink_set_vehicle_position(
    BoinkHandle handle, 
    uint64_t vehicle_id, 
    const struct BoinkVec3* position)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(position==nullptr)
    return BOINK_ERR_INVALID_ARG;

  boink::Vehicle* p_vehicle;
  HANDLE_EXCEPTIONS(
    p_vehicle=&p_sim->getVehicle(vehicle_id));
  
  btVector3 pos(position->x,position->y,position->z);
  p_vehicle->setPosition(pos);

  return BOINK_OK;
}

int boink_set_track_position(BoinkHandle handle,const struct BoinkVec3* position)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(position==nullptr)
    return BOINK_ERR_INVALID_ARG;

  btVector3 pos(position->x,position->y,position->z);
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(pos);
  p_sim->getTrack().setWorldTransform(trans);

  return BOINK_OK;
}

int boink_read_vehicle_state(
    BoinkHandle handle, uint64_t vehicle_id, BoinkVehicleState *out_state)
{
  boink::Simulation* p_sim=(boink::Simulation*)handle;
  if(p_sim==nullptr)
    return BOINK_ERR_INVALID_ARG;
  if(out_state==nullptr)
    return BOINK_ERR_INVALID_ARG;
  
  boink::Vehicle* p_vehicle=nullptr;
  HANDLE_EXCEPTIONS(
    p_vehicle=&p_sim->getVehicle(vehicle_id))

  out_state->engine_rpm=0.0;
  out_state->gear=0;
  out_state->wheel_speeds[0]=0.0;
  out_state->wheel_speeds[1]=0.0;
  out_state->wheel_speeds[2]=0.0;
  out_state->wheel_speeds[3]=0.0;
  out_state->brake_applied=0.0;
  out_state->throttle_applied=0.0;

  out_state->vehicle_id=vehicle_id;
  out_state->speed=p_vehicle->getSpeed();

  btTransform chassis_transform=p_vehicle->getChassisWorldTransform();
  out_state->chassis_position=bt2boink(chassis_transform.getOrigin());
  out_state->vehicle_orientation=bt2boink(chassis_transform.getRotation());
  
  btTransform wheel_transform;

  wheel_transform=p_vehicle->getWheelWorldTransform(boink::WheelPosition::FrontLeft);
  out_state->wheel_position[0]=bt2boink(wheel_transform.getOrigin());
  //out_state->wheel_orientation[0]=bt2boink(wheel_transform.getRotation());

  wheel_transform=p_vehicle->getWheelWorldTransform(boink::WheelPosition::FrontRight);
  out_state->wheel_position[1]=bt2boink(wheel_transform.getOrigin());
  //out_state->wheel_orientation[1]=bt2boink(wheel_transform.getRotation());

  wheel_transform=p_vehicle->getWheelWorldTransform(boink::WheelPosition::RearLeft);
  out_state->wheel_position[2]=bt2boink(wheel_transform.getOrigin());
  //out_state->wheel_orientation[2]=bt2boink(wheel_transform.getRotation());

  wheel_transform=p_vehicle->getWheelWorldTransform(boink::WheelPosition::RearRight);
  out_state->wheel_position[3]=bt2boink(wheel_transform.getOrigin());
  //out_state->wheel_orientation[3]=bt2boink(wheel_transform.getRotation());

  return BOINK_OK;
}

int exceptionType2api(boink::Exception::Type type)
{
  switch(type)
  {
    case boink::Exception::Type::InvalidArgumentError:
      return BOINK_ERR_INVALID_ARG;
    case boink::Exception::Type::UnsupportedFormatError:
      return BOINK_ERR_UNSUPPORTED_FORMAT;
    case boink::Exception::Type::IOError:
      return BOINK_ERR_IO;
    case boink::Exception::Type::NotFoundError:
      return BOINK_ERR_NOT_FOUND;
  }

  return BOINK_ERR_INTERNAL;
}

BoinkVec3 bt2boink(btVector3 bt_vec)
{
  return {bt_vec.getX(),bt_vec.getY(),bt_vec.getZ()};
}

//btVector3 boink2bt(BoinkVec3 boink_vec)
//{
//  return btVector3(boink_vec.x,boink_vec.y,boink_vec.z);
//}

BoinkQuaternion bt2boink(btQuaternion bt_quat)
{
  return {bt_quat.getX(),bt_quat.getY(),bt_quat.getZ(),bt_quat.getW()};
}

//btQuaternion boink2bt(BoinkQuaternion boink_quat)
//{
//  return btQuaternion(boink_quat.x,boink_quat.y,boink_quat.z,boink_quat.w);
//}
