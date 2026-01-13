#pragma once

#include "boink/components/car_drive_parts.h"
#include "boink/components/car_inputs.h"
#include "boink/components/component_manager.h"
#include "boink/components/kinematics.h"
#include "boink/helpers/enums.h"

namespace boink
{
class DrivePartsSystem
{
  // this system should be calculated before movement system
 public:
  template <typename TupleStaticComponents_, typename TupleComponents_>
  void Update(ComponentManager<TupleStaticComponents_, TupleComponents_>& component_manager, double dt)
  {
    auto view = component_manager.template getComponentView<CarInput, CarDriveParts, Kinematics>();

    view.forEach([&, dt](const CarInput& input, CarDriveParts& parts, Kinematics& kin) 
    {
      auto speed = kin.velocity.norm();
      auto slip_ratio = (parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed * Wheel::radius - speed) / speed;
      kin.traction_force = kin.dynamic_rear_weight * kSlipRatioToGrip.GetValue(slip_ratio);

      auto drive_torque = parts.engine.GetMaxTorque() * input.throttle * parts.gearbox.GetCurrentRatio() *
                          Gearbox::kDifferentialRatio * kTransmissionEfficiency;
      auto traction_torque = -kin.traction_force * kWheelRadius;
      auto brake_torque = 0;  // TODO: implement brake torque
      auto total_wheel_torque = drive_torque + traction_torque + brake_torque;
      auto wheel_inertia = kWheelMass * kWheelRadius * kWheelRadius * kWheelMassDistCoeff;
      auto wheel_angular_acceleration = total_wheel_torque / (2 * wheel_inertia);  // for two driven wheels
      auto wheel_speed_diff = wheel_angular_acceleration * dt;
      parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed += wheel_speed_diff;
      parts.wheels[static_cast<int>(WheelIndex::RR)].angular_speed += wheel_speed_diff;

      // feedback to engine
      auto new_rpm = parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed * parts.gearbox.GetCurrentRatio() *
                     Gearbox::kDifferentialRatio * (60.0f / (2.0f * 3.14159f));
    }
  }

 private:
  static constexpr float kTransmissionEfficiency = 0.7f;
  static constexpr float kWheelRadius = 0.33f;
  static constexpr float kWheelMass = 16.0f;
  static constexpr float kWheelMassDistCoeff = 0.78f;

  static constexpr Curve kSlipRatioToGrip =
      Curve({0.000, 0.850, 1.100, 1.080, 1.020, 0.970, 0.930, 0.900, 0.880, 0.865, 0.850,
             0.840, 0.830, 0.825, 0.820, 0.815, 0.810, 0.805, 0.800, 0.800, 0.800},
            0.05f, 0.0f);
}
}  // namespace boink