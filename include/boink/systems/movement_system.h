#pragma once

#include <cmath>
#include <iostream>

#include "Eigen/Core"
#include "boink/components/car_drive_parts.h"
#include "boink/components/car_inputs.h"
#include "boink/components/car_model.h"
#include "boink/components/component_manager.h"
#include "boink/components/kinematics.h"
#include "boink/components/transform.h"
#include "boink/utils/math.h"

namespace boink
{
/**
 * @brief Simulates movement of a vehicle
 *
 */
class MovementSystem
{
 public:
  /**
   * @brief Updates all entities with Transform and Velocity components.
   *
   * @tparam Components_ The component types to that belong to
   * ComponentManager.
   *
   * @param component_manager Reference to the ComponentManager storing
   * components.
   * @param dt Delta time.
   */
  template <typename TupleStaticComponents_, typename TupleComponents_>
  void Update(ComponentManager<TupleStaticComponents_, TupleComponents_>& component_manager, double dt)
  {
    auto view = component_manager.template getComponentView<CarInput, Transform, Kinematics, CarDriveParts>();

    auto static_comps = component_manager.template getStaticComponentView<CarModel>();
    const auto& model = std::get<CarModel&>(static_comps);

    view.forEach(
        [&, dt](const CarInput& input, Transform& trans, Kinematics& kin, CarDriveParts& parts)
        {
          using namespace Eigen;

          // Assumes that Y axis points to graviational force.
          // Assumes the right hand coordinates system.
          // Assumes that during delta time car moves with constant acceleration.
          // Assumes that postive angle means roaton counterclockwise.
          // this means that angle_x is postive when rotation is from Y to Z, for
          // angle_y is from Z to X and for angle_z from X to Y.

          // Assumes that initial car postion is at 0,0,0 if not displacement
          // vector must be provided
          //
          // Assumes breaking is just accelerting in oppostie dir

          // Steer_angle >0 -> left; <0 -> right

          auto acceleration = kin.traction_force / kMass;
          kin.velocity = (kin.velocity.norm() + acceleration * dt) * wheel_direction;

          auto weight_shift = (Kinematics::kCoGHeight / Kinematics::kWheelBase) * acceleration * kMass;
          kin.dynamic_front_weight = Kinematics::kStationaryFrontWeight - weight_shift;
          kin.dynamic_rear_weight = Kinematics::kStationaryRearWeight + weight_shift;

          /*double traction_force = input.throttle * parts.engine.power;
          double drag_force = -kAeroDrag * std::pow(kin.velocity.norm(), 2);
          double rolling_resistance_force = -kRollingResistance * kin.velocity.norm();
          double braking_force = -input.brake * kBrakingFactor;
          double longtitudinal_force = traction_force + braking_force + drag_force + rolling_resistance_force;

          kin.acceleration = longtitudinal_force / kMass;*/

          double turn_angle = math::deg2rad(input.steer_angle * model.max_steer_angle_deg);

          double sin_turn = std::sin(turn_angle);
          double cos_turn = std::cos(turn_angle);
          Matrix3d turn_rotation = math::getRodriguesRotationMatrix(sin_turn, cos_turn, model.normal);
          Vector3d wheel_direction = turn_rotation * model.direction;

          // kin.velocity = (kin.velocity.norm() + kin.acceleration * dt) * wheel_direction;

          std::cout << "acceleration: " << acceleration << "\n";
          std::cout << "speed: " << kin.velocity.norm() << "\n";

          Vector3d front_displacement = kin.velocity * dt + (0.5 * acceleration * dt * dt) * wheel_direction;

          Vector3d new_model_direction = (model.direction + front_displacement);
          new_model_direction.normalize();
          Matrix3d delta_rotation = math::getRotationMatrix(model.direction, new_model_direction);

          Vector3d new_front = model.front + front_displacement;
          Vector3d rot_front = delta_rotation * model.front;
          Vector3d front_new_disp_after_d_rot = new_front - rot_front;

          // Its rotation matrix so traspose equals to inverse.
          trans.position = trans.rotation.transpose() * front_new_disp_after_d_rot + trans.position;

          // Order matters
          trans.rotation = trans.rotation * delta_rotation;
        });
  }

 private:
  // TODO: mass should be move to new chassis component, now it is doubled in kinematics.h
  static constexpr float kMass = 650.0f;
  static constexpr double kAeroDrag = 0.4257;
  static constexpr double kRollingResistance = 12.8;
  static constexpr double kBrakingFactor = 10000;
};
}  // namespace boink
