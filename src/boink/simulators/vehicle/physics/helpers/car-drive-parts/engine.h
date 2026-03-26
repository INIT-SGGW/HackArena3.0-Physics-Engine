#pragma once

#include <vector>

#include "boink/simulators/vehicle/physics/helpers/curve.h"

namespace boink
{
struct Engine
{
  bool is_revLimiter_active = false;
  bool m_is_on_idle = true;
  float inertia = 0.5f;  // [kg*m^2]
  float rpm = 4000.0f;
  btScalar m_idle_timer = 0.f;

  btScalar GetIdleRPM() const
  {
    float targetIdle = 4000.0f;

    // 1. ECU Hunting (Wolne falowanie komputera walczacego o utrzymanie obrotow)
    // Zmienia sie powoli, np. co 1-2 sekundy. Amplituda: +/- 40 RPM
    float ecuHunt = sin(m_idle_timer * 2.5f) * 40.0f;

    // 2. Mechaniczna asymetria (Szybsze falowanie od walkow rozrzadu)
    // Zmienia sie szybko. Amplituda: +/- 60 RPM
    float mechVibe = sin(m_idle_timer * 12.0f) * 60.0f;

    // 3. Wypadanie zaplonow / Szum (Micro-noise)
    // Bardzo szybkie, losowe szarpniecia. Amplituda: +/- 20 RPM
    // (rand() % 100) / 50.0f - 1.0f generuje losowego float'a od -1.0 do 1.0
    float noise = (((rand() % 100) / 50.0f) - 1.0f) * 20.0f;

    return targetIdle + ecuHunt + mechVibe + noise;
  }

  void UpdateIdleRPMTimer(btScalar timeStep) { m_idle_timer += timeStep; }

  /// <summary>
  /// Returns maximum torque at current RPM (max is when a throttle is fully open)
  /// </summary>
  /// <returns>
  /// Maximum torque in [Nm]
  /// </returns>
  btScalar GetMaxTorque() { return RPM_to_torque.GetValue(rpm); }

  void SetNewRPM(btScalar new_rpm)
  {
    if (new_rpm > 16000.f)
      rpm = 16000.f;
    else
      rpm = new_rpm;
  }

 private:
  static inline const Curve RPM_to_torque = Curve(
      {200.f, 250.f, 320.f, 380.f, 440.f, 500.f, 550.f, 590.f, 620.f, 640.f, 660.f, 670.f, 675.f, 678.f, 680.f, 680.f,
       680.f, 680.f, 675.f, 670.f, 660.f, 650.f, 635.f, 620.f, 600.f, 580.f, 560.f, 535.f, 510.f, 480.f, 450.f},
      500.0f, 0.0f);  // N*m
};
}  // namespace boink
