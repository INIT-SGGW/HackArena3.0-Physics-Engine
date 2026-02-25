#include "boink/boink_c_api.h"

#include "boink/debugger/debugger.h"
#include "boink/exception.h"
#include "boink/simulation/race.h"
#include "boink/simulation/simulation.h"
#include "boink/simulators/vehicle/vehicle.h"
#include "boink/simulators/vehicle/vehicle_mesh.h"

#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/version.h"

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <exception>
#include <memory>
#include <numbers>
#include <sstream>

#define RETURN_STATUS(x) \
    {\
    set_last_error(\
        __func__,#x,\
        nullptr);\
    return x;\
    }

#define RETURN_CODE_STR(x) #x

#define RETURN_STATUS_INVALID_ARG(arg,reason) \
    {\
    set_last_error(\
        __func__,\
        RETURN_CODE_STR(BOINK_ERR_INVALID_ARG),\
        #arg " " reason);\
    return BOINK_ERR_INVALID_ARG;\
    }

#define IF_RETURN_STATUS_INVALID_ARG_NULL(arg) \
          if(arg==nullptr)\
  RETURN_STATUS_INVALID_ARG(arg, "was null")

#define HANDLE_EXCEPTIONS(block)       \
try {                                  \
    block;                              \
} catch (const boink::Exception& e) {  \
    int code=exceptionType2api(e.getType()); \
    set_last_error(\
        __func__,\
        returnCodeStr(code),\
        e.what());\
    return code;\
} catch (const std::exception& e) {    \
    set_last_error(\
        __func__,\
        returnCodeStr(BOINK_ERR_INTERNAL),\
        e.what());\
    return BOINK_ERR_INTERNAL;          \
} catch (...) {                         \
    RETURN_STATUS(BOINK_ERR_INTERNAL);\
}

static int exceptionType2api(boink::Exception::Type type);
static BoinkVec3 bt2boink(btVector3 bt_vec);
//static btVector3 boink2bt(BoinkVec3 boink_vec);
static BoinkQuaternion bt2boink(btQuaternion bt_quat);
//static btQuaternion boink2bt(BoinkQuaternion boink_quat);

static boink::Debugger* gp_dbg=nullptr;
static thread_local std::string g_last_error="";

static void set_last_error(const char* function, const char* return_code_str,const char* opt_desc);
static const char* returnCodeStr(int code);

int boink_get_c_api_version(unsigned int *out_major,
                                 unsigned int *out_minor,
                                 unsigned int *out_patch)
{
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_major);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_minor);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_patch);

  *out_major=BOINK_C_API_VERSION_MAJOR;
  *out_minor=BOINK_C_API_VERSION_MINOR;
  *out_patch=BOINK_C_API_VERSION_PATCH;

  return BOINK_OK;
}

int boink_get_engine_version(unsigned int *out_major,
                                 unsigned int *out_minor,
                                 unsigned int *out_patch)
{
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_major);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_minor);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_patch);

  *out_major=BOINK_VERSION_MAJOR;
  *out_minor=BOINK_VERSION_MINOR;
  *out_patch=BOINK_VERSION_PATCH;

  return BOINK_OK;
}

int boink_get_engine_profile(char* out_buf, unsigned int* in_out_len)
{
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      in_out_len);

#ifdef NDEBUG
  const char* profile_name="release";
#else
  const char* profile_name="debug";
#endif
  unsigned int required_size=strlen(profile_name)+1;

  if(!out_buf || *in_out_len<required_size)
  {
    *in_out_len=required_size;
    RETURN_STATUS(BOINK_ERR_BUFFER_TOO_SMALL);
  }

  memcpy(out_buf,profile_name,required_size);
  *in_out_len=required_size;

  return BOINK_OK;
}

int boink_get_last_error(char* out_buf, unsigned int* in_out_len)
{
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      in_out_len);

  const char* error_desc=g_last_error.c_str();
  unsigned int required_size=g_last_error.length()+1;

  if(!out_buf || *in_out_len<required_size)
  {
    *in_out_len=required_size;
    return BOINK_ERR_BUFFER_TOO_SMALL;
  }

  memcpy(out_buf,error_desc,required_size);
  *in_out_len=required_size;
  return BOINK_OK;
}

void set_last_error(const char* function, const char* return_code_string,const char* opt_desc)
{
  std::stringstream ss;
  ss<<"Error type: "<<return_code_string<<std::endl;
  ss<<"Function: "<<function;
  if(opt_desc)
  {
    ss<<std::endl<<"Description: ";
    ss<<opt_desc;
  }

  g_last_error=ss.str();
}

int boink_init(bool debug_drawer_enable)
{
  if(debug_drawer_enable)
  {
    HANDLE_EXCEPTIONS(
      gp_dbg=new boink::Debugger(
        "Boink Debugger",
        {0.f,0.f,1.f},
        {0.f,0.f,0.f}))
  }
  return BOINK_OK;
}

void boink_terminate()
{
  if(gp_dbg!=nullptr)
  {
    delete gp_dbg;
    gp_dbg=nullptr;
  }
}

BoinkHandle boink_create_race(const char* track_glb_filename)
{
  if(track_glb_filename==nullptr)
  {
    set_last_error(
        __func__,
        RETURN_CODE_STR(BOINK_ERR_INVALID_ARG),
        "track_glb_filename was null");
    return nullptr;
  }
  try
  {
    boink::Race* p_race=new boink::Race(9.71f,track_glb_filename,gp_dbg);
    
    return reinterpret_cast<BoinkHandle>(p_race);
  }
  catch(boink::Exception& e)
  {
    int code=exceptionType2api(e.getType());
    set_last_error(
        __func__,
        returnCodeStr(code),
        e.what());
    return nullptr;
  }
  catch (const std::exception& e) 
  {
    set_last_error(
        __func__,
        RETURN_CODE_STR(BOINK_ERR_INTERNAL),
        e.what());
    return nullptr;
  }
  catch(...)
  {
    set_last_error(
        __func__,
        RETURN_CODE_STR(BOINK_ERR_INTERNAL),
        nullptr);
    return nullptr;
  }
}

void boink_destroy_race(BoinkHandle handle)
{
  if(handle!=nullptr)
    delete (boink::Race*) handle;
}

int boink_create_vehicle_mesh(
    const char* glb_model_filename,
    BoinkVehicleMeshHandle* out_mesh_handle)
{
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      glb_model_filename);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_mesh_handle);

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
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_dur);

  *out_dur=p_sim->getSimulationDuration();
  return BOINK_OK;
}

int boink_step_race(BoinkHandle handle, Real dt)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  if(dt<0.)
    RETURN_STATUS_INVALID_ARG(
        dt,"was lesser than 0");

  p_race->update(dt);
  p_race->updateDebug();
  return BOINK_OK;
}

void boink_update_debug()
{
  if(gp_dbg)
  {
    gp_dbg->getRendererPtr()->drawFrameOrigin();
    gp_dbg->update();
  }
}

Real boink_get_time_debug()
{
  if(gp_dbg)
    return gp_dbg->getTime();

  return 0;
}

bool boink_should_close_debug()
{
  if(gp_dbg)
    return gp_dbg->shouldClose();

  return true;
}

int boink_spawn_vehicle(
    BoinkHandle handle, 
    const BoinkVehicleModel* p_vehicle_model,
    uint64_t* out_vehicle_id)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      p_vehicle_model);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_vehicle_id);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      p_vehicle_model->mesh);

  boink::Vehicle::CreationInfo create_info;
  create_info.center_of_mass.setX(p_vehicle_model->center_of_mass.x);
  create_info.center_of_mass.setY(p_vehicle_model->center_of_mass.y);
  create_info.center_of_mass.setZ(p_vehicle_model->center_of_mass.z);
  
  create_info.mass=p_vehicle_model->mass;
  create_info.wheel_radius=p_vehicle_model->wheel_radius;
  create_info.suspension_rest_length=p_vehicle_model->suspension_rest_length;
  create_info.max_steer_angle =
      p_vehicle_model->max_steer_angle * btScalar(std::numbers::pi / 180.0);

  create_info.mesh=std::shared_ptr<const boink::VehicleMesh>(
      (const boink::VehicleMesh*)p_vehicle_model->mesh,
      [](const boink::VehicleMesh*){});

  create_info.tuning.m_frictionSlip=3.5f;
  create_info.tuning.m_maxSuspensionForce=20000.f;
  create_info.tuning.m_maxSuspensionTravelCm=8.f;
  create_info.tuning.m_suspensionStiffness=75.f;
  create_info.tuning.m_suspensionDamping=2.5f;
  create_info.tuning.m_suspensionCompression=2.5f;

  // TODO
  // I think try is not needed here but it must be checked
  HANDLE_EXCEPTIONS(
    *out_vehicle_id=p_race->addVehicle(create_info))

  return BOINK_OK;
}

int boink_despawn_vehicle(BoinkHandle handle, uint64_t vehicle_id)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);

  HANDLE_EXCEPTIONS(
    p_race->removeVehicle(vehicle_id))
  
  return BOINK_OK;
}

int boink_set_controls(
    BoinkHandle handle, 
    uint64_t vehicle_id, 
    const BoinkControls* controls)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      controls);
  
  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id))

  if(controls->throttle>1. || controls->throttle<0.)
  {
    RETURN_STATUS_INVALID_ARG(
        controls->throttle,
        "was lesser than 0 or greater than 1");
  }
    
  if(controls->brake>1. || controls->brake<0.)
  {
    RETURN_STATUS_INVALID_ARG(
        controls->brake,
        "was lesser than 0 or greater than 1");
  }

  if(controls->steer>1. || controls->steer<-1.)
  {
    RETURN_STATUS_INVALID_ARG(
        controls->steer,
        "was lesser than -1 or greater than 1");
  }

  vehicle->setEngineForce(controls->throttle);
  vehicle->setBrake(controls->brake);

  Real steer=std::abs(controls->steer);
  boink::Vehicle::TurnDirection dir=
    controls->steer<0.0?
    boink::Vehicle::TurnDirection::Left:
    boink::Vehicle::TurnDirection::Right;
  vehicle->setSteering(steer,dir);

  return BOINK_OK;
}

int boink_set_vehicle_position(
    BoinkHandle handle, 
    uint64_t vehicle_id, 
    const struct BoinkVec3* position)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      position);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));
  
  btVector3 pos(position->x,position->y,position->z);
  vehicle->setPosition(pos);

  return BOINK_OK;
}

int boink_set_track_position(BoinkHandle handle,const struct BoinkVec3* position)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      position);

  btVector3 pos(position->x,position->y,position->z);
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(pos);
  p_race->getTrack()->setWorldTransform(trans);

  return BOINK_OK;
}

int boink_read_vehicle_state(
    BoinkHandle handle, uint64_t vehicle_id, BoinkVehicleState *out_state)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_state);
  
  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id))

  out_state->engine_rpm=0.0;
  out_state->gear=0;
  for(int i=0;i<4;i++)
  {
    btScalar angular_speed=vehicle->getWheelAngularSpeed((boink::WheelPosition)i);
    out_state->wheel_speeds[i]=angular_speed;
  }
  out_state->brake_applied=0.0;
  out_state->throttle_applied=0.0;

  out_state->vehicle_id=vehicle_id;
  out_state->speed=vehicle->getSpeed();

  btTransform chassis_transform=vehicle->getChassisWorldTransform();
  out_state->chassis_position=bt2boink(chassis_transform.getOrigin());
  out_state->vehicle_orientation=bt2boink(chassis_transform.getRotation());
  
  btTransform wheel_transform;

  // TODO 
  // maybe change name to front wheels orientation?
  // and add rear wheel orientation?

  wheel_transform=vehicle->getWheelWorldTransform(boink::WheelPosition::FrontLeft);
  out_state->wheel_position[0]=bt2boink(wheel_transform.getOrigin());

  wheel_transform=vehicle->getWheelWorldTransform(boink::WheelPosition::FrontRight);
  out_state->wheel_position[1]=bt2boink(wheel_transform.getOrigin());

  wheel_transform=vehicle->getWheelWorldTransform(boink::WheelPosition::RearLeft);
  out_state->wheel_position[2]=bt2boink(wheel_transform.getOrigin());

  wheel_transform=vehicle->getWheelWorldTransform(boink::WheelPosition::RearRight);
  out_state->wheel_position[3]=bt2boink(wheel_transform.getOrigin());

  return BOINK_OK;
}

int boink_set_weather(BoinkHandle handle, const BoinkWeather* weather)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      weather);
  (void)p_race;

  Real transition_duration=30.f;
  
  auto weather_sim=p_race->getWeather();
  weather_sim->setTemperatureCelcius(weather->temperature_c,transition_duration);
  weather_sim->setCloudiness(weather->cloudiness,transition_duration);
  weather_sim->setRainIndensity(weather->rain_intensity,transition_duration);

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
    case boink::Exception::Type::InternalError:
      return BOINK_ERR_INTERNAL;
  }

  return BOINK_ERR_INTERNAL;
}

#define RETURN_CODE_CASE(x) case x: return #x;

const char* returnCodeStr(int code)
{
    switch (code)
    {
        RETURN_CODE_CASE(BOINK_OK)
        RETURN_CODE_CASE(BOINK_ERR_INVALID_ARG)
        RETURN_CODE_CASE(BOINK_ERR_BUFFER_TOO_SMALL)
        RETURN_CODE_CASE(BOINK_ERR_NOT_FOUND)
        RETURN_CODE_CASE(BOINK_ERR_UNSUPPORTED_FORMAT)
        RETURN_CODE_CASE(BOINK_ERR_IO)
        RETURN_CODE_CASE(BOINK_ERR_INTERNAL)

        default:
            return "Unknown return code string";
    }
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
