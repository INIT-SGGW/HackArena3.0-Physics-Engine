#pragma once

#include <cmath>

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

    view.forEach(
        [&, dt](const CarInput& input, CarDriveParts& parts, Kinematics& kin)
        {
          // gearbox stuff
          auto speed = kin.velocity.norm();
          auto current_gear = static_cast<uint8_t>(parts.gearbox.current_gear);
          if (input.gear_up)
          {
            auto new_rpms = -1.f;

            if (current_gear == static_cast<uint8_t>(Gear::Neutral))
            {
              new_rpms = parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed *
                         parts.gearbox.kGearRatios[current_gear + 1] * Gearbox::kDifferentialRatio *
                         (60.0f / (2.0f * 3.14159f));
            }
            else if (current_gear != (std::size(parts.gearbox.kGearRatios) - 1))
            {
              new_rpms = parts.engine.rpm *
                         (parts.gearbox.kGearRatios[current_gear + 1] / parts.gearbox.GetCurrentRatio());
            }

            if (new_rpms >= 0)
            {
              parts.engine.SetNewRPM(new_rpms);
              parts.gearbox.current_gear = static_cast<Gear>(current_gear + 1);
            }
          }

          if (input.gear_down)
          {
            if (current_gear == static_cast<uint8_t>(Gear::Neutral))
            {
              if (speed == 0) parts.gearbox.current_gear = static_cast<Gear>(current_gear - 1);
            }
            else if (current_gear != 0)
            {
              auto new_rpms = parts.engine.rpm *
                              (parts.gearbox.kGearRatios[current_gear - 1] / parts.gearbox.GetCurrentRatio());
              if (new_rpms < 15000)
              {
                parts.engine.SetNewRPM(new_rpms);
                parts.gearbox.current_gear = static_cast<Gear>(current_gear - 1);
              }
            }
          }

          auto drive_torque = 0.0f;
          if (input.throttle == 0)
          {
            drive_torque =
                -70 * parts.gearbox.GetCurrentRatio() * Gearbox::kDifferentialRatio * kTransmissionEfficiency;
          }
          else if (!parts.engine.is_revLimiter_active)
          {
            drive_torque = parts.engine.GetMaxTorque() * input.throttle * parts.gearbox.GetCurrentRatio() *
                           Gearbox::kDifferentialRatio * kTransmissionEfficiency;
          }

          auto traction_torque = -kin.traction_force * kWheelRadius;
          auto brake_torque = (input.brake > 0) ? -10000 : 0;
          auto total_wheel_torque = drive_torque + traction_torque + brake_torque;

          auto wheel_inertia =
              kWheelMass * kWheelRadius * kWheelRadius * kWheelMassDistCoeff;  // TODO: make wheel inertia a
                                                                               // const literal in wheel.h
          auto engine_inertia_part =
              parts.engine.inertia *
              std::powf(
                  parts.gearbox.GetCurrentRatio() * parts.gearbox.kDifferentialRatio * kTransmissionEfficiency, 2);

          auto wheel_angular_acceleration = total_wheel_torque / (wheel_inertia * 2 + engine_inertia_part);
          auto wheel_speed_diff = wheel_angular_acceleration * dt;
          if (current_gear == static_cast<uint8_t>(Gear::Reverse))
          {
            if ((parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed + wheel_speed_diff) > 0)
            {
              parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed = 0;
              parts.wheels[static_cast<int>(WheelIndex::RR)].angular_speed = 0;
            }
            else
            {
              parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed += wheel_speed_diff;
              parts.wheels[static_cast<int>(WheelIndex::RR)].angular_speed += wheel_speed_diff;
            }
          }
          else
          {
            if ((parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed + wheel_speed_diff) < 0)
            {
              parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed = 0;
              parts.wheels[static_cast<int>(WheelIndex::RR)].angular_speed = 0;
            }
            else
            {
              parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed += wheel_speed_diff;
              parts.wheels[static_cast<int>(WheelIndex::RR)].angular_speed += wheel_speed_diff;
            }
          }

          // feedback to engine
          auto new_rpm = parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed *
                         parts.gearbox.GetCurrentRatio() * Gearbox::kDifferentialRatio *
                         (60.0f / (2.0f * 3.14159f));
          if (new_rpm > 15000.0f)  // rev limiter
            parts.engine.is_revLimiter_active = true;
          else if (parts.engine.is_revLimiter_active && new_rpm < 14600.0f)
            parts.engine.is_revLimiter_active = false;
          parts.engine.SetNewRPM(new_rpm);

          auto slip_ratio = 0.0f;

          if (speed == 0 && input.throttle == 0)
            kin.traction_force = 0.0f;
          else
          {
            if (speed == 0)
            {
              if (current_gear == static_cast<uint8_t>(Gear::Reverse))
                slip_ratio = -0.0001f;
              else
                slip_ratio = 0.0001f;
            }
            else if (speed < 2.9 && input.throttle == 0)
            {
              // this is a protection against speed approachig zero and then slip
              // ratio approaching infinity what causing numerical instability (
              // TODO: and not working too well xD)
              slip_ratio =
                  (parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed * Wheel::radius - speed) / 2.9;
            }
            else
            {
              slip_ratio =
                  (parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed * Wheel::radius - speed) / speed;
            }

            // downforce
            auto air_density = 1.225f;  // kg/m^3
            auto forntal_area = 1.5f;   // m^2
            auto downforce_coefficient = 3.5f;
            auto downforce = 0.5f * air_density * forntal_area * downforce_coefficient * speed * speed;
            auto downforce_rear = downforce * (kin.kWheelBase - kin.kCoG_to_rear) / kin.kWheelBase;

            if (slip_ratio < 0)
              kin.traction_force =
                  (kin.dynamic_rear_weight + downforce_rear) * -kSlipRatioToGrip.GetValue(-slip_ratio);
            else
              kin.traction_force =
                  (kin.dynamic_rear_weight + downforce_rear) * kSlipRatioToGrip.GetValue(slip_ratio);
          }

          std::cout << "gear:  " << parts.gearbox.current_gear << "\t";
          std::cout << "new_rpm:  " << new_rpm << "\t";
          std::cout << "drive_torque:  " << drive_torque << "\t";
          std::cout << "traction_torque: " << traction_torque << "\t";  // on old traction force
          std::cout << "total_torque: " << total_wheel_torque << "\t";
          std::cout << "wheel_ang_speed: " << parts.wheels[static_cast<int>(WheelIndex::RL)].angular_speed << "\t";
          std::cout << "old_speed: " << speed << "\t";
          std::cout << "slip_ratio: " << slip_ratio << "\t";
          std::cout << "traction_force: " << kin.traction_force << "\n";
        });
  }

 private:
  static constexpr float kTransmissionEfficiency = 0.7f;
  static constexpr float kWheelRadius = 0.33f;
  static constexpr float kWheelMass = 16.0f;
  static constexpr float kWheelMassDistCoeff = 0.78f;

  static inline const Curve kSlipRatioToGrip =
      Curve({0.000, 0.850, 1.100, 1.080, 1.020, 0.970, 0.930, 0.900, 0.880, 0.865, 0.850,
             0.840, 0.830, 0.825, 0.820, 0.815, 0.810, 0.805, 0.800, 0.800, 0.800},
            0.05f, 0.0f);
};
}  // namespace boink