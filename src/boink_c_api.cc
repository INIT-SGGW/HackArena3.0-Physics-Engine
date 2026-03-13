// clang-format off
#include "boink/boink_c_api.h"

#include "boink/debugger/debugger.h"
#include "boink/exception.h"
#include "boink/simulation/race.h"
#include "boink/simulation/simulation.h"
#include "boink/simulators/vehicle/ghost_mode_settings.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/vehicle.h"
#include "boink/simulators/vehicle/vehicle_mesh.h"

#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/version.h"
#include "boink/constants.h"

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btScalar.h>
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
  unsigned int required_size=(unsigned int)strlen(profile_name)+1;

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
  unsigned int required_size= (unsigned int)g_last_error.length()+1;

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
  if constexpr(sizeof(btScalar)!=sizeof(Real))
  {
    set_last_error(
        __func__,
        returnCodeStr(BOINK_ERR_INTERNAL),
        "btScalar is not the same type as Real");

    return BOINK_ERR_INTERNAL;
  }

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
  {
    delete[] (BoinkCenterlineSample*)((boink::Race*) handle)->getUserPtr();
    delete (boink::Race*) handle;
  }
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

int boink_get_track_data(BoinkHandle handle, BoinkTrackData *out_track_data)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_track_data);

  //if(sizeof(BoinkTrackData)!=sizeof(boink::Track::SampleData))
  //{
  //  set_last_error(
  //      __func__,
  //      returnCodeStr(BOINK_ERR_INTERNAL),
  //      "BoinkTrackData structure differs from boink::Track::SampleData");

  //  return BOINK_ERR_INTERNAL;
  //}

  auto& track_data=p_race->getTrack()->getTrackData();

  if(p_race->getUserPtr()==nullptr)
  {
    BoinkCenterlineSample* samples=new BoinkCenterlineSample[track_data.size()];
    for(size_t i=0;i<track_data.size();i++)
    {
      samples[i].bank_rad=track_data[i].bank;
      samples[i].curvature_1pm=track_data[i].curvature;
      samples[i].grade_rad=track_data[i].grade;
      samples[i].left_width_m=track_data[i].left_width;
      samples[i].right_width_m=track_data[i].right_width;
      samples[i].s_m=track_data[i].coverage;

      samples[i].normal.x=track_data[i].normal.getX();
      samples[i].normal.y=track_data[i].normal.getY();
      samples[i].normal.z=track_data[i].normal.getZ();

      samples[i].position.x=track_data[i].position.getX();
      samples[i].position.y=track_data[i].position.getY();
      samples[i].position.z=track_data[i].position.getZ();

      samples[i].right.x=track_data[i].right.getX();
      samples[i].right.y=track_data[i].right.getY();
      samples[i].right.z=track_data[i].right.getZ();

      samples[i].tangent.x=track_data[i].tangent.getX();
      samples[i].tangent.y=track_data[i].tangent.getY();
      samples[i].tangent.z=track_data[i].tangent.getZ();
    }
    p_race->setUserPtr(samples);
  }

  out_track_data->map_id=p_race->getTrack()->getFilename().data();
  out_track_data->version=0;
  out_track_data->lap_length_m=p_race->getTrack()->getCenterline().getLength();
  out_track_data->centerline_samples=
    reinterpret_cast<BoinkCenterlineSample*>(p_race->getUserPtr());
  out_track_data->centerline_sample_count= (unsigned int)
    p_race->getTrack()->getTrackData().size();

  return BOINK_OK;
}

int boink_step_race(BoinkHandle handle, Real dt, Real* out_simulated_dt_second)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_simulated_dt_second);
  if(dt<0.)
    RETURN_STATUS_INVALID_ARG(
        dt,"was lesser than 0");

  int max_sub_steps=15;
  Real fixed_delta_time=1.f/120.f;
  Real max_delta_time=0.1f;
  int steps=p_race->update(dt,max_sub_steps,fixed_delta_time,max_delta_time);
  *out_simulated_dt_second=steps*fixed_delta_time;

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

  create_info.tyre_type=boink::WheelInfo::TyreType::Wet;

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
    const BoinkControls* controls,
    BoinkAcceptedControls* out_accepted_controls)
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

  if(controls->gear_shift < 0 || controls->gear_shift > 2)
  {
    RETURN_STATUS_INVALID_ARG(
        controls->gear_shift,
        "was lesser than 0 or greater than 2");
  }

  vehicle->setEngineForce(controls->throttle);
  vehicle->setBrake(controls->brake);

  Real steer=std::abs(controls->steer);
  boink::Vehicle::TurnDirection dir=
    controls->steer<0.0?
    boink::Vehicle::TurnDirection::Left:
    boink::Vehicle::TurnDirection::Right;
  vehicle->setSteering(steer,dir);

  out_accepted_controls->accepted_shift = BoinkGearShift::BOINK_GEAR_SHIFT_NONE;
  switch (controls->gear_shift)
  {
    case BoinkGearShift::BOINK_GEAR_SHIFT_UPSHIFT:
      if (vehicle->setGearUp()) out_accepted_controls->accepted_shift = BoinkGearShift::BOINK_GEAR_SHIFT_UPSHIFT;
      break;
    case BoinkGearShift::BOINK_GEAR_SHIFT_DOWNSHIFT:
      if (vehicle->setGearDown()) out_accepted_controls->accepted_shift = BoinkGearShift::BOINK_GEAR_SHIFT_DOWNSHIFT;
      break;
    case BoinkGearShift::BOINK_GEAR_SHIFT_NONE:;
  }

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
  btTransform transform=vehicle->getChassisWorldTransform();
  transform.setOrigin(pos);
  vehicle->setChassisWorldTransform(transform);

  return BOINK_OK;
}

int boink_set_vehicle_before_point(
    BoinkHandle handle,
    uint64_t vehicle_id,
    const BoinkVec3* point)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      point);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));
  
  btVector3 bt_point(point->x,point->y,point->z);

  btVector3 bt_forward=p_race->getTrack()->getForwardDirection(bt_point);
  boink::Vehicle::Dimensions bounding_dims=vehicle->getBoundingDims();


  btVector3 axis_rot=boink::g_Forward.cross(bt_forward);
  btScalar rot_angle=boink::g_Forward.angle(bt_forward);

  btQuaternion rot;

  if(axis_rot.length2()<boink::g_Epsilon)
  {
    if(boink::g_Forward.dot(bt_forward)>0.f)
      rot=btQuaternion::getIdentity();
    else
      rot=btQuaternion(boink::g_Up*-1,SIMD_PI);
  }
  else
  {
    axis_rot.normalize();
    rot=btQuaternion(axis_rot,rot_angle);
  }

  btTransform transform=vehicle->getChassisWorldTransform();
  transform.setRotation(rot);
  vehicle->setChassisWorldTransform(transform);

  btVector3 up_compensate=boink::g_Up*vehicle->getChassisToGroundDist();
  up_compensate=quatRotate(rot,up_compensate);

  btScalar half_depth=bounding_dims.depth/2.f;
  btVector3 bt_pos=-1*bt_forward*half_depth+bt_point;
  bt_pos+=up_compensate;

  BoinkVec3 pos;
  pos.x=bt_pos.x();
  pos.y=bt_pos.y();
  pos.z=bt_pos.z();
  HANDLE_EXCEPTIONS(
      boink_set_vehicle_position(handle,vehicle_id,&pos));

  return BOINK_OK;
}

int boink_set_vehicle_before_finish_line(
    BoinkHandle handle,
    uint64_t vehicle_id)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  btVector3 bt_finish_point=p_race->getTrack()->getFinishLine();

  BoinkVec3 finish_point;
  finish_point.x=bt_finish_point.x();
  finish_point.y=bt_finish_point.y();
  finish_point.z=bt_finish_point.z();
  HANDLE_EXCEPTIONS(
      boink_set_vehicle_before_point(
        handle,vehicle_id,&finish_point));

  return BOINK_OK;
}

int boink_set_vehicle_random_pos(BoinkHandle handle,uint64_t vehicle_id)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  btVector3 bt_random_pos=p_race->getTrack()->getOnTrackRandomPosition();
  BoinkVec3 random_pos;
  random_pos.x=bt_random_pos.x();
  random_pos.y=bt_random_pos.y();
  random_pos.z=bt_random_pos.z();
  HANDLE_EXCEPTIONS(
      boink_set_vehicle_before_point(
        handle,vehicle_id,&random_pos));

  return BOINK_OK;
}

int boink_set_vehicle_orientation(
    BoinkHandle handle,
    uint64_t vehicle_id,
    const struct BoinkQuaternion *orientation)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      orientation);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  btQuaternion rot(
      orientation->x,
      orientation->y,
      orientation->z,
      orientation->w);
  btTransform transform=vehicle->getChassisWorldTransform();
  vehicle->setChassisWorldTransform(transform*btTransform(rot));

  return BOINK_OK;
}


int boink_set_vehicle_at_start_pos(
    BoinkHandle handle,
    uint64_t vehicle_id,
    uint64_t position_index)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  btVector3 bt_start_pos;
  HANDLE_EXCEPTIONS(
    bt_start_pos=p_race->getTrack()->getStartingPosition(position_index));

  btVector3 bt_forward=p_race->getTrack()->getForwardDirection(bt_start_pos);
  boink::Vehicle::Dimensions bounding_dims=vehicle->getBoundingDims();
  btScalar half_depth=bounding_dims.depth/2.f;
  bt_start_pos=bt_forward*half_depth+bt_start_pos;

  BoinkVec3 start_pos;
  start_pos.x=bt_start_pos.x();
  start_pos.y=bt_start_pos.y();
  start_pos.z=bt_start_pos.z();

  HANDLE_EXCEPTIONS(
      boink_set_vehicle_before_point(
        handle,vehicle_id,&start_pos));

  return BOINK_OK;
}

int boink_get_number_of_start_pos(BoinkHandle handle,uint64_t* out_number_pos)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_number_pos);

  *out_number_pos=p_race->getTrack()->getNumberOfStartingPositions();

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

  out_state->engine_rpm=vehicle->getEngineRPM();
  out_state->gear=vehicle->getCurrentGear() - 1;

  out_state->wheel_speeds[0] = vehicle->getWheelAngularSpeed(boink::WheelPosition::FrontLeft);
  out_state->wheel_speeds[1] = vehicle->getWheelAngularSpeed(boink::WheelPosition::FrontRight);
  out_state->wheel_speeds[2] = vehicle->getWheelAngularSpeed(boink::WheelPosition::RearLeft);
  out_state->wheel_speeds[3] = vehicle->getWheelAngularSpeed(boink::WheelPosition::RearRight);
  
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

  Real transition_duration=30.f;
  
  auto weather_sim=p_race->getWeather();
  weather_sim->setTemperatureCelcius(weather->temperature_c,transition_duration);
  weather_sim->setCloudiness(weather->cloudiness,transition_duration);
  weather_sim->setRainIndensity(weather->rain_intensity,transition_duration);

  return BOINK_OK;
}

int boink_set_ghost_mode_settings(
    BoinkHandle handle, const BoinkGhostModeSettings* settings)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      settings);

  boink::GhostModeSettings boink_settings;
  boink_settings.enabled_until_completed_laps=settings->until_completed_laps;
  boink_settings.enter_delay=settings->enter_delay_ms/1000.f;
  boink_settings.exit_delay=settings->exit_delay_ms/1000.f;
  boink_settings.exit_delay_when_overlap=settings->vehicle_overlap_exit_delay_ms/1000.f;
  boink_settings.max_enter_speed=settings->enter_speed_max_mps;
  boink_settings.min_exit_speed=settings->exit_speed_min_mps;

  if(boink_settings.min_exit_speed<boink_settings.max_enter_speed)
  {
    RETURN_STATUS_INVALID_ARG(
        settings.min_exit_speed,
        "cannot be lower than settings.max_enter_speed");
  }

  HANDLE_EXCEPTIONS(
    p_race->enableGhostMode(boink_settings));
  return BOINK_OK;
}

int boink_disable_ghost_mode(BoinkHandle handle)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);

  p_race->disableGhostMode();

  return BOINK_OK;
}
int boink_read_vehicle_ghost_mode_state(
    BoinkHandle handle,
    uint64_t vehicle_id,
    struct BoinkGhostModeRuntimeState *out_state)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id))
  
  const auto& ghost_mode=vehicle->getGhostMode();
  const auto& enter_timer=ghost_mode.getEnterTimer();
  const auto& exit_timer=ghost_mode.getExitTimer();
  const auto& overlap_timer=ghost_mode.getOverlapTimer();

  BoinkGhostModeRuntimeState state;
  state.can_collide_now=!vehicle->isInGhostMode();

  state.blockers_mask=0;

  state.blockers_mask|=
    ghost_mode.isCompletedLapsConditionMet()?
    BOINK_GHOST_MODE_BLOCKER_LAPS_REQUIREMENT_NOT_MET:
    0;

  state.blockers_mask|=
    !ghost_mode.isExitSpeedConditionMet()?
    BOINK_GHOST_MODE_BLOCKER_EXIT_SPEED_NOT_MET:
    0;

  state.blockers_mask|=
    exit_timer.isRunning()?
    BOINK_GHOST_MODE_BLOCKER_EXIT_DELAY_RUNNING:
    0;

  state.blockers_mask|=
    ghost_mode.isOverlapping()?
    BOINK_GHOST_MODE_BLOCKER_VEHICLE_OVERLAP_ACTIVE:
    0;

  state.blockers_mask|=
    overlap_timer.isRunning()?
    BOINK_GHOST_MODE_BLOCKER_OVERLAP_EXIT_DELAY_RUNNING:
    0;

  if(state.blockers_mask==0 && ghost_mode.isInGhostMode())
  {
    set_last_error(
        __func__,
        RETURN_CODE_STR(BOINK_ERR_INTERNAL),
        "out_state->blockers_mask is 0 but vehicle is in ghost mode");
    return BOINK_ERR_INTERNAL;
  }

  state.exit_delay_remaining_ms=0;
  state.enter_delay_remaining_ms=0;
  if(ghost_mode.isInGhostMode())
  {
    int mask=0;
    mask|=BOINK_GHOST_MODE_BLOCKER_LAPS_REQUIREMENT_NOT_MET;
    mask|=BOINK_GHOST_MODE_BLOCKER_EXIT_SPEED_NOT_MET;
    mask|=BOINK_GHOST_MODE_BLOCKER_VEHICLE_OVERLAP_ACTIVE;
    mask|=BOINK_GHOST_MODE_BLOCKER_IN_PIT;

    if((mask&state.blockers_mask)==0)
    {
      if(exit_timer.isRunning())
        state.exit_delay_remaining_ms=
          (unsigned int)((exit_timer.getTarget()-exit_timer.getCurrent())*1000);

      if(overlap_timer.isRunning())
      {
        unsigned int exit_delay_overlap_ms=
          (unsigned int)((overlap_timer.getTarget()-overlap_timer.getCurrent())*1000);

        if(exit_delay_overlap_ms>state.exit_delay_remaining_ms)
          state.exit_delay_remaining_ms=exit_delay_overlap_ms;
      }
      state.phase=BOINK_GHOST_MODE_PHASE_PENDING_EXIT;
    }
    else
    {
      state.exit_delay_remaining_ms=0;
      state.phase=BOINK_GHOST_MODE_PHASE_ACTIVE;
    }
  }
  else
  {
    if(enter_timer.isRunning())
    {
      state.enter_delay_remaining_ms=
        (unsigned int)((enter_timer.getTarget()-enter_timer.getCurrent())*1000);

      state.phase=BOINK_GHOST_MODE_PHASE_PENDING_ENTER;
    }
    else
    {
      state.enter_delay_remaining_ms=0;
      state.phase=BOINK_GHOST_MODE_PHASE_INACTIVE;
    }
  } 

  *out_state=state;
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
