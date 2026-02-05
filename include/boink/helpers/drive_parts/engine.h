#pragma once

#include <iostream>
#include <vector>

#include "boink/helpers/curve.h"

namespace boink
{
struct Engine
{
  bool is_revLimiter_active = false;
  float inertia = 0.5f;  // [kg*m^2]
  float rpm = 4000.0f;

  /// <summary>
  /// Returns maximum torque at current RPM (max is when a throttle is fully open)
  /// </summary>
  /// <returns>
  /// Maximum torque in [Nm]
  /// </returns>
  float GetMaxTorque() { return RPM_to_torque.GetValue(rpm); }

  void SetNewRPM(float new_rpm)
  {
    if (new_rpm <= 4000)
      rpm = 4000.0f;
    else
      rpm = new_rpm;
    // std::cout << "rpm: " << rpm << "\n";
  }

 private:
  static inline const Curve RPM_to_torque =
      Curve({0,   40,  80,  100, 120, 135, 150, 165, 180, 210, 240, 255, 270, 283, 295, 305, 315, 323, 330, 338,
             345, 350, 355, 360, 365, 369, 372, 375, 378, 380, 382, 384, 385, 384, 380, 372, 360, 345, 320, 300},
            500.0f, 0.0f);

  // RPM validation is in the Curve class, but here can be implemented rev limiter
};
}  // namespace boink