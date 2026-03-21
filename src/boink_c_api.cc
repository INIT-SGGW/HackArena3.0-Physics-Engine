// clang-format off
#include "boink/boink_c_api.h"
#include "boink/fmt_boink_c_api.h"

#include "boink/debugger/debugger.h"
#include "boink/exception.h"
#include "boink/logger.h"
#include "boink/simulation/race.h"
#include "boink/simulation/simulation.h"
#include "boink/simulators/vehicle/ghost_mode_settings.h"
#include "boink/simulators/vehicle/lap_info.h"
#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/vehicle.h"
#include "boink/simulators/vehicle/vehicle_mesh.h"

#include "boink/simulators/vehicle/wheel_position.h"
#include "boink/version.h"
#include "boink/constants.h"
#include "boink/assert.h"

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>
#include <exception>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <random>

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

struct EngineData
{
  size_t main_samples_size;
  BoinkCenterlineSample* main_samples;
  size_t entry_samples_size;
  BoinkCenterlineSample* entry_pitstop_samples;
  size_t fix_samples_size;
  BoinkCenterlineSample* fix_pitstop_samples;
  size_t exit_samples_size;
  BoinkCenterlineSample* exit_pitstop_samples;

  ~EngineData()
  {
    // Delete nested arrays for main_samples
    for(size_t i = 0; i < main_samples_size; ++i) {
      delete[] main_samples[i].left_grounds;
      delete[] main_samples[i].right_grounds;
    }
    delete[] main_samples;

    // Delete nested arrays for entry_pitstop_samples
    for(size_t i = 0; i < entry_samples_size; ++i) {
      delete[] entry_pitstop_samples[i].left_grounds;
      delete[] entry_pitstop_samples[i].right_grounds;
    }
    delete[] entry_pitstop_samples;

    // Delete nested arrays for fix_pitstop_samples
    for(size_t i = 0; i < fix_samples_size; ++i) {
      delete[] fix_pitstop_samples[i].left_grounds;
      delete[] fix_pitstop_samples[i].right_grounds;
    }
    delete[] fix_pitstop_samples;

    // Delete nested arrays for exit_pitstop_samples
    for(size_t i = 0; i < exit_samples_size; ++i) {
      delete[] exit_pitstop_samples[i].left_grounds;
      delete[] exit_pitstop_samples[i].right_grounds;
    }
    delete[] exit_pitstop_samples;
  }
};

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

  if(!out_buf)
  {
    *in_out_len=required_size;
    RETURN_STATUS(BOINK_OK);
  }

  if(*in_out_len<required_size)
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

  if(!out_buf)
  {
    *in_out_len=required_size;
    RETURN_STATUS(BOINK_OK);
  }

  if(*in_out_len<required_size)
  {
    *in_out_len=required_size;
    RETURN_STATUS(BOINK_ERR_BUFFER_TOO_SMALL);
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

  boink::Logger::init();
  BOINK_INFO("Initializing engine");
  BOINK_INFO("C API version: ({}.{}.{})",
      BOINK_C_API_VERSION_MAJOR,
      BOINK_C_API_VERSION_MINOR,
      BOINK_C_API_VERSION_PATCH);
  BOINK_INFO("Engine version: ({}.{}.{})",
      BOINK_VERSION_MAJOR,
      BOINK_VERSION_MINOR,
      BOINK_VERSION_PATCH);

  if(debug_drawer_enable)
  {
    BOINK_INFO("Debug drawer enabled");
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
  BOINK_INFO("Terminating engine");
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
    
    BOINK_INFO("Created race with track file: {}",track_glb_filename);
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
    delete (EngineData*)((boink::Race*) handle)->getUserPtr();
    delete (boink::Race*) handle;
  }
  BOINK_INFO("Destroyed race");
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

  BOINK_INFO("Created vehicle mesh with model file: {}",glb_model_filename);
  return BOINK_OK;
}

void boink_destroy_vehicle_mesh(BoinkVehicleMeshHandle handle)
{
  if(handle!=nullptr)
    delete reinterpret_cast<boink::VehicleMesh*>(handle);
  BOINK_INFO("Destroyed vehicle mesh");
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

BoinkCenterlineSample* createBoinkCenterlineSamples(
    const boink::Road& road,
    size_t* out_size)
{
  const auto& track_data=road.getRoadData();
  BoinkCenterlineSample* samples=new BoinkCenterlineSample[track_data.size()];
  for(size_t i=0;i<track_data.size();i++)
  {
    const btVector3& position=road.getPoint(i);
    btScalar coverage=road.getCoverage(position);

    samples[i].bank_rad=track_data[i].bank;
    samples[i].curvature_1pm=track_data[i].curvature;
    samples[i].grade_rad=track_data[i].grade;
    samples[i].left_width_m=track_data[i].left_width;
    samples[i].right_width_m=track_data[i].right_width;
    samples[i].max_left_width_m=track_data[i].left_max_width;
    samples[i].max_right_width_m=track_data[i].right_max_width;
    samples[i].s_m=coverage;

    samples[i].normal.x=track_data[i].normal.getX();
    samples[i].normal.y=track_data[i].normal.getY();
    samples[i].normal.z=track_data[i].normal.getZ();

    samples[i].position.x=position.getX();
    samples[i].position.y=position.getY();
    samples[i].position.z=position.getZ();

    samples[i].right.x=track_data[i].right.getX();
    samples[i].right.y=track_data[i].right.getY();
    samples[i].right.z=track_data[i].right.getZ();

    samples[i].tangent.x=track_data[i].tangent.getX();
    samples[i].tangent.y=track_data[i].tangent.getY();
    samples[i].tangent.z=track_data[i].tangent.getZ();

    if(track_data[i].left_grounds.size()>0)
    {
      samples[i].left_grounds_count=(unsigned int)track_data[i].left_grounds.size();
      samples[i].left_grounds=new BoinkGroundWidth[samples[i].left_grounds_count];
      for(size_t j=0;j<track_data[i].left_grounds.size();j++)
      {
        samples[i].left_grounds[j].width=track_data[i].left_grounds[j].first;
        samples[i].left_grounds[j].type=
          static_cast<BoinkGroundType>(track_data[i].left_grounds[j].second);
      }
    }
    else
    {
      samples[i].left_grounds_count=0;
      samples[i].left_grounds=nullptr;
    }
    if(track_data[i].right_grounds.size()>0)
    {
      samples[i].right_grounds_count=(unsigned int)track_data[i].right_grounds.size();
      samples[i].right_grounds=new BoinkGroundWidth[samples[i].right_grounds_count];
      for(size_t j=0;j<track_data[i].right_grounds.size();j++)
      {
        samples[i].right_grounds[j].width=track_data[i].right_grounds[j].first;
        samples[i].right_grounds[j].type=
          static_cast<BoinkGroundType>(track_data[i].right_grounds[j].second);
      }
    }
    else
    {
      samples[i].right_grounds_count=0;
      samples[i].right_grounds=nullptr;
    }

  }
  *out_size=track_data.size();

  return samples;
}

int boink_get_track_data(BoinkHandle handle, BoinkTrackData *out_track_data)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_track_data);

  const auto& track=p_race->getTrack();
  const auto& main_road=track->getRoad();;

  if(p_race->getUserPtr()==nullptr)
  {
    EngineData* p_engine_data=new EngineData();
    const auto& pitstop=track->getPitstop();

    BoinkCenterlineSample* main_samples=
      createBoinkCenterlineSamples(main_road,&p_engine_data->main_samples_size);
    BoinkCenterlineSample* entry_pitstop_samples=
      createBoinkCenterlineSamples(
          pitstop.getZone(boink::Pitstop::Zone::Enter),
          &p_engine_data->entry_samples_size);
    BoinkCenterlineSample* fix_pitstop_samples=
      createBoinkCenterlineSamples(
          pitstop.getZone(boink::Pitstop::Zone::Fix),
          &p_engine_data->fix_samples_size);
    BoinkCenterlineSample* exit_pitstop_samples=
      createBoinkCenterlineSamples(
          pitstop.getZone(boink::Pitstop::Zone::Exit),
          &p_engine_data->exit_samples_size);

    p_engine_data->main_samples=main_samples;
    p_engine_data->entry_pitstop_samples=entry_pitstop_samples;
    p_engine_data->fix_pitstop_samples=fix_pitstop_samples;
    p_engine_data->exit_pitstop_samples=exit_pitstop_samples;

    p_race->setUserPtr(p_engine_data);
  }

  EngineData* p_engine_data=reinterpret_cast<EngineData*>(p_race->getUserPtr());

  out_track_data->map_id=track->getFilename().data();
  out_track_data->lap_length_m=main_road.getLength();
  out_track_data->pitstop_data.length_m=track->getPitstop().getLength();

  int version=track->getTrackVersion();
  out_track_data->version=
    version>=0?
    version:
    std::numeric_limits<decltype(out_track_data->version)>::max();

  out_track_data->centerline_samples=p_engine_data->main_samples;
  out_track_data->centerline_sample_count=p_engine_data->main_samples_size;

  out_track_data->pitstop_data.enter_centerline_samples=
    p_engine_data->entry_pitstop_samples;
  out_track_data->pitstop_data.enter_centerline_sample_count=
    p_engine_data->entry_samples_size;

  out_track_data->pitstop_data.fix_centerline_samples=
    p_engine_data->fix_pitstop_samples;
  out_track_data->pitstop_data.fix_centerline_sample_count=
    p_engine_data->fix_samples_size;

  out_track_data->pitstop_data.exit_centerline_samples=
    p_engine_data->exit_pitstop_samples;
  out_track_data->pitstop_data.exit_centerline_sample_count=
    p_engine_data->exit_samples_size;

  // Debug logging for track data
  BOINK_TRACE("=== Track Data ===");
  BOINK_TRACE("map_id: {}", out_track_data->map_id);
  BOINK_TRACE("version: {}", out_track_data->version);
  BOINK_TRACE("lap_length_m: {}", out_track_data->lap_length_m);
  BOINK_TRACE("centerline_sample_count: {}", out_track_data->centerline_sample_count);
  
  BOINK_TRACE("=== Main Centerline Samples ===");
  for(unsigned int i = 0; i < out_track_data->centerline_sample_count; ++i) {
    const auto& sample = out_track_data->centerline_samples[i];
    BOINK_TRACE("[{}] s_m: {}, pos: {}, tangent: {}, normal: {}, right: {}, "
                "left_width: {}, right_width: {}, max_left_width: {}, max_right_width: {}, "
                "curvature: {}, grade: {}, bank: {}",
                i, sample.s_m,
                sample.position, sample.tangent, sample.normal, sample.right,
                sample.left_width_m, sample.right_width_m,
                sample.max_left_width_m, sample.max_right_width_m,
                sample.curvature_1pm, sample.grade_rad, sample.bank_rad);
    
    BOINK_TRACE("  left_grounds_count: {}, right_grounds_count: {}",
                sample.left_grounds_count, sample.right_grounds_count);
    for(unsigned int j = 0; j < sample.left_grounds_count; ++j) {
      BOINK_TRACE("    left_grounds[{}]: width={}, type={}", j,
                  sample.left_grounds[j].width, sample.left_grounds[j].type);
    }
    for(unsigned int j = 0; j < sample.right_grounds_count; ++j) {
      BOINK_TRACE("    right_grounds[{}]: width={}, type={}", j,
                  sample.right_grounds[j].width, sample.right_grounds[j].type);
    }
  }
  
  BOINK_TRACE("=== Pitstop Data ===");
  BOINK_TRACE("pitstop_length_m: {}", out_track_data->pitstop_data.length_m);
  BOINK_TRACE("enter_centerline_sample_count: {}", out_track_data->pitstop_data.enter_centerline_sample_count);
  BOINK_TRACE("fix_centerline_sample_count: {}", out_track_data->pitstop_data.fix_centerline_sample_count);
  BOINK_TRACE("exit_centerline_sample_count: {}", out_track_data->pitstop_data.exit_centerline_sample_count);

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

btQuaternion get_rotation_relative_to_track(
    const btVector3& bt_forward,const btVector3& bt_normal)
{
  btQuaternion align_to_surface = shortestArcQuat(boink::g_Up, bt_normal);
  btVector3 local_forward = quatRotate(align_to_surface, boink::g_Forward);

  btQuaternion align_to_tangent = shortestArcQuat(local_forward, bt_forward);
  btQuaternion final_rot = align_to_tangent * align_to_surface;

  return final_rot;
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

  btVector3 bt_forward=
    p_race->
    getTrack()->
    getRoad().getClosestMetrics(bt_point).tangent;
  btVector3 bt_normal=
    p_race->
    getTrack()->
    getRoad().getClosestMetrics(bt_point).normal;

  btQuaternion rot=get_rotation_relative_to_track(bt_forward,bt_normal);

  btTransform transform=vehicle->getChassisWorldTransform();
  transform.setRotation(rot);
    vehicle->setChassisWorldTransform(transform);

  btVector3 up_compensate=
    boink::g_Up*(vehicle->getChassisToGroundDist()+boink::g_GroundMargin);
  up_compensate=quatRotate(rot,up_compensate);

  boink::BoundingBox bounding_dims=vehicle->getBoundingDims();
  btScalar half_depth=(bounding_dims.top_left-bounding_dims.bottom_left).length()/2.f;
  btVector3 bt_pos=-1*bt_forward*half_depth+bt_point;
  bt_pos+=up_compensate;

  transform=vehicle->getChassisWorldTransform();
  transform.setOrigin(bt_pos);
  vehicle->setChassisWorldTransform(transform);

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


  std::random_device rd;
  thread_local static std::mt19937 gen(rd());
  std::uniform_real_distribution<btScalar> dist(0,2*SIMD_PI);

repeat:
  const auto& road=p_race->getTrack()->getRoad();
  btVector3 bt_random_pos=road.getRandomPosition();
  auto sample=road.getClosestMetrics(bt_random_pos);
  btScalar angle=dist(gen);

  btQuaternion align;
  align = shortestArcQuat(boink::g_Up, sample.normal);

  btQuaternion yaw(sample.normal, angle);

  btQuaternion final_rot = yaw * align;

  btVector3 offset=vehicle->getCenterOfMassCS();
  if(4!=road.isObjectOnRoad(
        bt_random_pos,
        final_rot,
        offset,
        vehicle->getBoundingDims()))
    goto repeat;

  btTransform bt_transform = vehicle->getChassisWorldTransform();
  bt_transform.setRotation(final_rot);
  vehicle->setChassisWorldTransform(bt_transform);

  btVector3 up_compensate=
    sample.normal*(vehicle->getChassisToGroundDist()+boink::g_GroundMargin);
  bt_random_pos+=up_compensate;

  btTransform transform=vehicle->getChassisWorldTransform();
  transform.setOrigin(bt_random_pos);
  vehicle->setChassisWorldTransform(transform);

  return BOINK_OK;
}

int boink_set_vehicle_back_to_track(BoinkHandle handle, uint64_t vehicle_id)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  btVector3 vehicle_pos=vehicle->getChassisWorldTransform().getOrigin();
  btVector3 new_pos=p_race->getTrack()->getRoad().getInterpolatedPoint1(vehicle_pos);
  
  const auto& metrics=p_race->getTrack()->getRoad().getClosestMetrics(new_pos);

  btVector3 up_compensate=
    metrics.normal*(vehicle->getChassisToGroundDist()+boink::g_GroundMargin);
  new_pos+=up_compensate;
  
  btQuaternion final_rot=get_rotation_relative_to_track(metrics.tangent,metrics.normal);

  btTransform bt_transform;
  bt_transform.setIdentity();
  bt_transform = vehicle->getChassisWorldTransform();
  bt_transform.setOrigin(new_pos);
  bt_transform.setRotation(final_rot);
  vehicle->setChassisWorldTransform(bt_transform);

  return BOINK_OK;
}

int boink_set_vehicle_to_pitstop(BoinkHandle handle, uint64_t vehicle_id)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  vehicle->setVehicleToPitstop(boink::Pitstop::Zone::Fix);
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

  btVector3 bt_forward=
    p_race->
    getTrack()->
    getRoad().getClosestMetrics(bt_start_pos).tangent;
  boink::BoundingBox bounding_dims=vehicle->getBoundingDims();

  btScalar half_depth=(bounding_dims.top_left-bounding_dims.bottom_left).length()/2.f;
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

  if(vehicle->hasStopped())
    out_state->speed=0.f;
  else
    out_state->speed=vehicle->getSpeed();

  btTransform chassis_transform=vehicle->getChassisWorldTransform();
  out_state->chassis_position=bt2boink(chassis_transform.getOrigin());
  out_state->vehicle_orientation=bt2boink(chassis_transform.getRotation());
  
  auto set_wheel_state=
  [&](boink::WheelPosition wheel_pos, size_t wheel_index)
  {
    btTransform wheel_transform=vehicle->getWheelWorldTransform(wheel_pos);
    out_state->wheel_position[wheel_index]=bt2boink(wheel_transform.getOrigin());
    out_state->tyre_temprature_celsius[wheel_index]=vehicle->getTyreTempCelsius(wheel_pos);
    out_state->wheel_speeds[wheel_index]=
      vehicle->getWheelAngularSpeed(wheel_pos);

    if(wheel_index<2){
      auto pair=vehicle->getSteering(wheel_pos);
      out_state->front_wheel_orientation_rad[wheel_index]=
        pair.second==boink::Vehicle::TurnDirection::Left?
        pair.first*-1:
        pair.first;
    }

  };

  out_state->are_all_wheels_on_ground=vehicle->areAllWheelsOnGround();

  set_wheel_state(boink::WheelPosition::FrontLeft,0);
  set_wheel_state(boink::WheelPosition::FrontRight,1);
  set_wheel_state(boink::WheelPosition::RearLeft,2);
  set_wheel_state(boink::WheelPosition::RearRight,3);

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

int boink_read_vehicle_race_metrics(
    BoinkHandle handle,
    uint64_t vehicle_id,
    struct BoinkVehicleRaceMetrics *out_metrics)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_metrics);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  const boink::LapInfo& lap_info=vehicle->getLapInfo();

  out_metrics->completed_laps=lap_info.getLapsCompleted();
  out_metrics->current_lap_time_ms=(unsigned int)(lap_info.curr_lap_time*1000.f);
  out_metrics->lap_progress_m=lap_info.curr_lap_coverage;

  auto opt_last_time=lap_info.getLastLapTime();
  if(opt_last_time.has_value())
  {
    out_metrics->last_lap_time_ms=
      (unsigned int)(opt_last_time.value().second*1000.f);

    out_metrics->has_last_lap_time=true;
  }
  else
  {
    out_metrics->last_lap_time_ms=0;
    out_metrics->has_last_lap_time=false;
  }

  return BOINK_OK;
}


int boink_get_vehicle_personal_best_lap(
    BoinkHandle handle,
    uint64_t vehicle_id,
    unsigned int *out_lap,
    unsigned int *out_lap_time_ms)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_lap);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_lap_time_ms);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  auto opt_best=vehicle->getLapInfo().getPersonalBest();

  if(!opt_best.has_value())
    return BOINK_NO_DATA;

  *out_lap=opt_best.value().first;
  *out_lap_time_ms=opt_best.value().second*1000;

  return BOINK_OK;
}

int boink_get_best_lap(
    BoinkHandle handle,
    uint64_t *out_vehicle_id,
    unsigned int *out_lap,
    unsigned int *out_lap_time_ms)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_vehicle_id);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_lap);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_lap_time_ms);

  auto opt_best=p_race->getBestLap();

  if(!opt_best.has_value())
    return BOINK_NO_DATA;

  const auto& [lap,best_time,id]=opt_best.value();

  *out_vehicle_id=id;
  *out_lap=lap;
  *out_lap_time_ms=(unsigned int)(best_time*1000.f);

  return BOINK_OK;
}

//BOINK_API int boink_get_vehicle_laps_history(
//    BoinkHandle handle,
//    uint64_t vehicle_id,
//    unsigned int *out_laps,
//    unsigned int *out_lap_times_ms,
//    uint64_t *in_out_count)
//{
//  boink::Race* p_race=(boink::Race*)handle;
//  IF_RETURN_STATUS_INVALID_ARG_NULL(
//      handle);
//  IF_RETURN_STATUS_INVALID_ARG_NULL(
//      in_out_count);
//
//  if ((out_laps != nullptr) != (out_lap_times_ms != nullptr)) {
//  {
//    set_last_error(
//        __func__,
//        RETURN_CODE_STR(BOINK_ERR_INVALID_ARG),
//        "Only one of out_laps or out_laps_times_ms is null."
//        " Must be both or none");
//    return BOINK_ERR_INVALID_ARG;
//  }
//
//  std::shared_ptr<boink::Vehicle> vehicle;
//  HANDLE_EXCEPTIONS(
//    vehicle=p_race->getVehicle(vehicle_id))
//
//  if(out_laps==nullptr) // and out_lap_times but we already checked that
//  {
//    vehicle->getLapInfo().lap_times_history.size();
//  }
//
//}

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

  //state.blockers_mask|=
  //  ghost_mode.isOverlapping()?
  //  BOINK_GHOST_MODE_BLOCKER_VEHICLE_OVERLAP_ACTIVE:
  //  0;

  //state.blockers_mask|=
  //  overlap_timer.isRunning()?
  //  BOINK_GHOST_MODE_BLOCKER_OVERLAP_EXIT_DELAY_RUNNING:
  //  0;
  state.blockers_mask|=
    vehicle->isVehicleInPitstop(boink::Pitstop::Zone::Fix)>0?
    BOINK_GHOST_MODE_BLOCKER_IN_PIT:
    0;


  state.exit_delay_remaining_ms=0;
  state.enter_delay_remaining_ms=0;
  if(ghost_mode.isInGhostMode())
  {
    if(exit_timer.isRunning())
    {
      state.exit_delay_remaining_ms=
        (unsigned int)((exit_timer.getTarget()-exit_timer.getCurrent())*1000);

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

  if(state.blockers_mask==0 && ghost_mode.isInGhostMode() && ghost_mode.isActive())
  {
    BOINK_WARN("state.blockers_mask={}",state.blockers_mask);
    BOINK_WARN("out_state->blockers_mask is 0 but vehicle is in ghost mode");
  }

  *out_state=state;
  return BOINK_OK;
}

int boink_get_vehicle_pitstop_zone(
    BoinkHandle handle,
    uint64_t vehicle_id,
    enum BoinkPitstopZone *out_zone,
    int *out_wheels_num)
{
  boink::Race* p_race=(boink::Race*)handle;
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      handle);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_zone);
  IF_RETURN_STATUS_INVALID_ARG_NULL(
      out_wheels_num);

  std::shared_ptr<boink::Vehicle> vehicle;
  HANDLE_EXCEPTIONS(
    vehicle=p_race->getVehicle(vehicle_id));

  int total_wheels_num=0;
  BoinkPitstopZone zone=BOINK_PITSTOP_ZONE_NONE;
  if(int wheels_num=vehicle->isVehicleInPitstop(boink::Pitstop::Zone::Enter);
      wheels_num>0)
  {
    total_wheels_num+=wheels_num;
    zone=(BoinkPitstopZone)(zone|BOINK_PITSTOP_ZONE_ENTER);
  }
  if(int wheels_num=vehicle->isVehicleInPitstop(boink::Pitstop::Zone::Fix);
      wheels_num>0)
  {
    total_wheels_num+=wheels_num;
    zone=(BoinkPitstopZone)(zone|BOINK_PITSTOP_ZONE_FIX);
  }
  if(int wheels_num=vehicle->isVehicleInPitstop(boink::Pitstop::Zone::Exit);
      wheels_num>0)
  {
    total_wheels_num+=wheels_num;
    zone=(BoinkPitstopZone)(zone|BOINK_PITSTOP_ZONE_EXIT);
  }

  BOINK_ASSERT(!(zone==BOINK_PITSTOP_ZONE_NONE && total_wheels_num!=0));

  *out_zone=zone;
  *out_wheels_num=total_wheels_num;

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
