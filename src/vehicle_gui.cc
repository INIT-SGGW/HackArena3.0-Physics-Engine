#include "boink/gui/vehicle_gui.h"

#include <imgui.h>

#include "boink/simulators/vehicle/physics/wheel_info.h"
#include "boink/simulators/vehicle/vehicle.h"
#include "boink/simulators/vehicle/wheel_position.h"

namespace boink
{
  VehicleGui::VehicleGui(Vehicle* p_vehicle)
    :p_vehicle_(p_vehicle)
  {}

  static const char* tyreTypeToStr(WheelInfo::TyreType type);
  static const char* wheelPosToStr(WheelPosition pos);
  void VehicleGui::draw()
  {
    if(p_vehicle_==nullptr)
      return;

    RaycastVehicle::VehicleTuning* tunning=&p_vehicle_->tuning_;

    ImGui::Checkbox("Mesh enabled",&mesh_enabled);
    ImGui::Checkbox("Collider enabled",&collider_enabled);
    ImGui::Checkbox("Ghost mode",&p_vehicle_->user_data_.ghost_info->enabled);

    ImGui::SliderFloat(
        "Friction slip",&tunning->m_frictionSlip,0.f,10.f);
    ImGui::SliderFloat(
        "Max suspension force",&tunning->m_maxSuspensionForce,0.f,50000.f);
    ImGui::SliderFloat(
        "Max suspension travel",&tunning->m_maxSuspensionTravelCm,0.f,20.f);
    ImGui::SliderFloat(
        "Suspension compression",&tunning->m_suspensionCompression,0.f,100.f);
    ImGui::SliderFloat(
        "Suspension damping",&tunning->m_suspensionDamping,0.f,100.f);
    ImGui::SliderFloat(
        "Suspension stifness",&tunning->m_suspensionStiffness,0.f,100.f);

    // TODO IMPROVEMENT
    // this can be improved via usage of slider values but
    // it takes to much time so only change tuning when value has
    // changed
    p_vehicle_->setTuning(p_vehicle_->tuning_);

    ImGui::Text("Laps completed: %d",p_vehicle_->getLapsCompleted());
    ImGui::Text("Current lap coverage: %.2f [m]",
        p_vehicle_->getCurrentLapDistanceCovered());
    btVector3 chassis_position=
      p_vehicle_->getChassisWorldTransform().getOrigin();
    ImGui::Text("Chassis position: (%.2f,%.2f,%.2f) [m]",
        chassis_position[0],chassis_position[1],chassis_position[2]);
    ImGui::Text("Mass: %.2f [kg]",p_vehicle_->getMass());
    ImGui::Text("Speed: %.2f [m/s]",p_vehicle_->getSpeed());
    btVector3 center_of_mass_cs=p_vehicle_->getCenterOfMassCS();
    ImGui::Text("COM in CS: (%.2f,%.2f,%.2f) [m]",
        center_of_mass_cs[0],center_of_mass_cs[1],center_of_mass_cs[2]);

    ImGui::SliderFloat(
        "Wear rate soft",&WheelInfo::TyreInfo::s_softWearRatePerMin,0.f,0.1f);
    ImGui::SliderFloat(
        "Wear rate hard",&WheelInfo::TyreInfo::s_hardWearRatePerMin,0.f,0.1f);
    ImGui::SliderFloat(
        "Wear rate wet",&WheelInfo::TyreInfo::s_wetWearRatePerMin,0.f,0.1f);

    ImGui::SliderFloat("Slip ratio temp const",
        &WheelInfo::TyreInfo::s_slipRatioTempConstant,0.f,0.5f);
    ImGui::SliderFloat("Wheel speed temp const",
        &WheelInfo::TyreInfo::s_angularSpeedTempConstant,0.f,0.1f);

    ImGui::SliderFloat("Wheel speed temp cooling const",
        &WheelInfo::TyreInfo::s_angularSpeedTempCoolingConst,0.f,0.1f);

    for(int i=0;i<(int)WheelPosition::Count;i++)
    {
      WheelPosition pos=(WheelPosition)i;

      std::string wheel_title="Wheel ";
      wheel_title+=wheelPosToStr(pos);
      ImGui::PushID(wheel_title.c_str());

      if(ImGui::CollapsingHeader(wheel_title.c_str()))
        drawWheel(pos);

      ImGui::PopID();
    }
  }

  void VehicleGui::drawWheel(WheelPosition pos)
  {
    const WheelInfo& info=p_vehicle_->vehicle_->getWheelInfo((int)pos);

    btVector3 pos_v=p_vehicle_->getWheelWorldTransform(pos).getOrigin();
    ImGui::Text("Position: (%.2f,%.2f,%.2f) [m]",
        pos_v.getX(),pos_v.getY(),pos_v.getZ());
    ImGui::Text("Angular speed: %.2f [rad/sec]",
        info.m_wheelAngularSpeed);
    ImGui::Text("Slip ratio: %.2f",
        info.m_slipRatio);

    ImGui::Text("Tyre type: %s",
        tyreTypeToStr(info.m_tyreInfo.m_type));
    ImGui::Text("Tyre health: %.2f",
        info.m_tyreInfo.m_health);
    ImGui::Text("Tyre temp: %.2f [C]",
        info.m_tyreInfo.m_tempCelsius);
  }

  const char* tyreTypeToStr(WheelInfo::TyreType type)
  {
    switch(type)
    {
      case WheelInfo::TyreType::Hard:
        return "hard";
      case WheelInfo::TyreType::Soft:
        return "soft";
      case WheelInfo::TyreType::Wet:
        return "wet";
      default:
        btAssert(false && "Unknown TyreType");
        return "unknown";
    }
  }

  const char* wheelPosToStr(WheelPosition pos)
  {
    switch(pos)
    {
      case WheelPosition::FrontLeft:
        return "Front left";
      case WheelPosition::FrontRight:
        return "Front right";
      case WheelPosition::RearLeft:
        return "Rear left";
      case WheelPosition::RearRight:
        return "Rear right";
      default:
        btAssert(false && "Unknown WheelPosition");
        return "unknown";
    }
  }
}
