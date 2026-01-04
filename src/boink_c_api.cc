#include "boink/boink_c_api.h"

#include "boink/components/car_inputs.h"
#include "boink/components/car_model.h"
#include "boink/components/transform.h"

#include "boink/components/kinematics.h"

#include "boink/utils/math.h"

#include "boink/world.h"
#include "boink/version.h"

#include <utility>
#include <Eigen/Geometry>

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

int boink_init()
{
  return BOINK_OK;
}

static Eigen::Vector3d b_v3_2_e_v(const BoinkVec3& b_vec)
{
  return Eigen::Vector3d{b_vec.x,b_vec.y,b_vec.z};
}

BoinkHandle boink_create_world(const BoinkCarModel* car_model)
{
  if (car_model == nullptr)
    return nullptr;

  boink::CarModel model{"abra kadabra"};
  model.front_left_wheel=b_v3_2_e_v(car_model->front_left_wheel);
  model.front_right_wheel=b_v3_2_e_v(car_model->front_right_wheel);
  model.rear_left_wheel=b_v3_2_e_v(car_model->rear_left_wheel);
  model.rear_right_wheel=b_v3_2_e_v(car_model->rear_right_wheel);
  model.max_steer_angle_deg=car_model->max_steer_angle;

  return (BoinkHandle)new (std::nothrow) boink::World(std::move(model));
}

int boink_begin_world(
    BoinkHandle handle, double time)
{
  boink::World* p_world=(boink::World*)handle;
  if(p_world==nullptr)
    return BOINK_ERR_INVALID_ARG;

  p_world->start(time);

  return BOINK_OK;
}

int boink_step(BoinkHandle handle, double dt)
{
  boink::World* p_world=(boink::World*)handle;
  if(p_world==nullptr)
    return BOINK_ERR_INVALID_ARG;

  p_world->update(dt);
  return BOINK_OK;
}

int boink_spawn_car(BoinkHandle handle, uint64_t* out_car_id)
{
  boink::World* p_world=(boink::World*)handle;
  if(p_world==nullptr)
    return BOINK_ERR_INVALID_ARG;

  *out_car_id=p_world->car_manager.addCar();

  return BOINK_OK;
}

int boink_despawn_car(BoinkHandle handle, uint64_t car_id)
{
  boink::World* p_world=(boink::World*)handle;
  if(p_world==nullptr)
    return BOINK_ERR_INVALID_ARG;

  if(!p_world->car_manager.removeCar(car_id))
    return BOINK_ERR_NOT_FOUND;
  
  return BOINK_OK;
}

int boink_set_controls(
    BoinkHandle handle, uint64_t car_id, const struct BoinkControls *controls)
{
  boink::World* p_world=(boink::World*)handle;
  if(p_world==nullptr)
    return BOINK_ERR_INVALID_ARG;

  boink::CarInput input;
  input.brake=controls->brake;
  input.steer_angle=controls->steer;
  input.throttle=controls->throttle;

  if (p_world->car_manager.updateCar(car_id, std::move(input)))
    return BOINK_OK;
  else
    return BOINK_ERR_NOT_FOUND;
}


int boink_read_car_state(
    BoinkHandle handle, uint64_t car_id, struct BoinkCarState *out_state)
{
  boink::World* p_world=(boink::World*)handle;
  if(p_world==nullptr)
    return BOINK_ERR_INVALID_ARG;

  if (!p_world->car_manager.isCarPresent(car_id))
  {
    return BOINK_ERR_NOT_FOUND;
  }

  out_state->engine_rpm=0.0;
  out_state->gear=0;
  out_state->wheel_speeds[0]=0.0;
  out_state->wheel_speeds[1]=0.0;
  out_state->wheel_speeds[2]=0.0;
  out_state->wheel_speeds[3]=0.0;
  out_state->car_id=car_id;

  auto components_opt=
    p_world->car_manager.
    getCarComponents<boink::CarInput,boink::Transform,boink::Kinematics>(car_id);

  if (!components_opt)
  {
    return BOINK_ERR_INTERNAL;
  }

  const auto& [input, trans, kins] = components_opt.value();
 
  auto model=p_world->
    car_manager.getCarStaticComponents<boink::CarModel>();

  double max_angle=std::get<boink::CarModel&>(model).max_steer_angle_deg;

  out_state->brake_applied=input.brake;
  out_state->throttle_applied=input.throttle;
  double angle=boink::math::deg2rad(input.steer_angle/2.0*max_angle);

  out_state->wheel_angles[0]=angle;
  out_state->wheel_angles[1]=angle;
  out_state->speed=kins.velocity.norm();

  Eigen::Quaterniond quad(trans.rotation);

  out_state->orientation.x = quad.x();
  out_state->orientation.y = quad.y();
  out_state->orientation.z = quad.z();
  out_state->orientation.w = quad.w();

  out_state->position={trans.position.x(),trans.position.y(),trans.position.z()};
  
  return BOINK_OK;
}

void boink_destroy_world(BoinkHandle handle)
{
  if(handle!=nullptr)
    delete (boink::World*) handle;
}
