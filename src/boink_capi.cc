#include "boink/boink_capi.h"

#include <utility>

#include "boink/components/car_inputs.h"
#include "boink/components/car_model.h"
#include "boink/components/kinematics.h"
#include "boink/components/transform.h"
#include "boink/utils/math.h"
#include "boink/world.h"

int boink_init() { return BOINK_OK; }

static Eigen::Vector3d b_v3_2_e_v(const BoinkVec3& b_vec) {
  return Eigen::Vector3d{b_vec.x, b_vec.y, b_vec.z};
}

BoinkHandle boink_create_world(const BoinkCarModel* car_model) {
  boink::CarModel model{};
  model.front_left_wheel = b_v3_2_e_v(car_model->front_left_wheel);
  model.front_right_wheel = b_v3_2_e_v(car_model->front_right_wheel);
  model.rear_left_wheel = b_v3_2_e_v(car_model->rear_left_wheel);
  model.rear_right_wheel = b_v3_2_e_v(car_model->rear_right_wheel);
  model.max_steer_angle_deg = car_model->max_steer_angle;

  return (BoinkHandle) new (std::nothrow) boink::World(std::move(model));
}

int boink_begin_world(BoinkHandle handle, double time) {
  boink::World* p_world = (boink::World*)handle;
  if (p_world == nullptr) return BOINK_ERR_INVALID_ARG;

  p_world->start(time);

  return BOINK_OK;
}

int boink_step(BoinkHandle handle, double dt) {
  boink::World* p_world = (boink::World*)handle;
  if (p_world == nullptr) return BOINK_ERR_INVALID_ARG;

  p_world->update(dt);
  return BOINK_OK;
}

int boink_spawn_car(BoinkHandle handle, uint64_t* out_car_id) {
  boink::World* p_world = (boink::World*)handle;
  if (p_world == nullptr) return BOINK_ERR_INVALID_ARG;

  *out_car_id = p_world->car_manager.addCar();

  return BOINK_OK;
}

int boink_despawn_car(BoinkHandle handle, uint64_t car_id) {
  boink::World* p_world = (boink::World*)handle;
  if (p_world == nullptr) return BOINK_ERR_INVALID_ARG;

  if (!p_world->car_manager.removeCar(car_id)) return BOINK_ERR_NOT_FOUND;

  return BOINK_OK;
}

int boink_set_controls(BoinkHandle handle, uint64_t car_id,
                       const struct BoinkControls* controls) {
  boink::World* p_world = (boink::World*)handle;
  if (p_world == nullptr) return BOINK_ERR_INVALID_ARG;

  boink::CarInput input;
  input.brake = controls->brake;
  input.steer_angle = controls->steer;
  input.throttle = controls->throttle;

  p_world->car_manager.updateCar(car_id, std::move(input));

  return BOINK_OK;
}

int boink_read_car_state(BoinkHandle handle, uint64_t car_id,
                         struct BoinkCarState* out_state) {
  boink::World* p_world = (boink::World*)handle;
  if (p_world == nullptr) return BOINK_ERR_INVALID_ARG;

  out_state->engine_rpm = 0.0;
  out_state->gear = 0;
  out_state->wheel_speeds[0] = 0.0;
  out_state->wheel_speeds[1] = 0.0;
  out_state->wheel_speeds[2] = 0.0;
  out_state->wheel_speeds[3] = 0.0;
  out_state->car_id = car_id;

  const auto& [input, trans, kins] =
      p_world->car_manager.getCarComponents<boink::CarInput, boink::Transform,
                                            boink::Kinematics>(car_id);

  auto model = p_world->car_manager.getCarStaticComponents<boink::CarModel>();

  double max_angle = std::get<boink::CarModel&>(model).max_steer_angle_deg;

  out_state->brake_applied = input.brake;
  out_state->throttle_applied = input.throttle;
  double angle = boink::math::deg2rad(input.steer_angle / 2.0 * max_angle);

  out_state->wheel_angles[0] = angle;
  out_state->wheel_angles[1] = angle;
  out_state->speed = kins.velocity.norm();

  Eigen::Vector3d rpy = trans.rotation.canonicalEulerAngles(2, 1, 0);
  out_state->orientation.roll = rpy(0);
  out_state->orientation.pitch = rpy(1);
  out_state->orientation.yaw = rpy(2);

  out_state->position = {trans.position.x(), trans.position.y(),
                         trans.position.z()};

  return BOINK_OK;
}

void boink_destroy_world(BoinkHandle handle) {
  if (handle != nullptr) delete (boink::World*)handle;
}
