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
    Curve({ 200.f, 250.f, 320.f, 380.f, 440.f, 500.f, 550.f, 590.f, 620.f, 640.f, 660.f, 670.f,
           675.f, 678.f, 680.f, 680.f, 680.f, 680.f, 675.f, 670.f, 660.f, 650.f, 635.f, 620.f,
           600.f, 580.f, 560.f, 535.f, 510.f, 480.f, 450.f },
      500.0f, 0.0f);  // N*m
};
}  // namespace boink
