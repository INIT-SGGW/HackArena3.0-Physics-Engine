#pragma once

#include <vector>

#include "boink/simulators/vehicle/physics/helpers/curve.h"

namespace boink
{
struct Engine
{
  bool is_revLimiter_active = false;
  btScalar inertia = 0.5f;  // [kg*m^2]
  btScalar rpm = 4000.0f;

  /// <summary>
  /// Returns maximum torque at current RPM (max is when a throttle is fully open)
  /// </summary>
  /// <returns>
  /// Maximum torque in [Nm]
  /// </returns>
  btScalar GetMaxTorque() { return RPM_to_torque.GetValue(rpm); }

  void SetNewRPM(btScalar new_rpm)
  {
    /*if (new_rpm <= 4000)
      rpm = 4000.0f;
    else*/
    rpm = new_rpm;
    // std::cout << "rpm: " << rpm << "\n";
  }

 private:
  static inline const Curve RPM_to_torque =
      Curve({200, 250, 320, 380, 440, 500, 550, 590, 620, 640, 660, 670, 675, 678, 680, 680,
             680, 680, 675, 670, 660, 650, 635, 620, 600, 580, 560, 535, 510, 480, 450},
            500.0f, 0.0f);  // N*m
};
}  // namespace boink
