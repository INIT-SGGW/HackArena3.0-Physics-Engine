#include "boink/debug_info_gui.h"

#include <sstream>
#include <iomanip>

namespace boink
{
  static inline std::ostream& operator<<(std::ostream& os, const btVector3& v)
  {
    os << '(' << v.x() << ", " << v.y() << ", " << v.z() << ')';
      return os;
  }
  
  void DebugInfoGui::draw()
  {
    GuiObject::checkBox("Freeze simulation",&freeze_sim_);
    GuiObject::checkBox("Enable camera",&enable_camera_);
    GuiObject::slider("Mouse speed",0.f,1.f,&mouse_speed_);
    GuiObject::slider("Camera speed",0.f,10.f,&camera_speed_);
    GuiObject::text("");

    std::stringstream ss;
    ss<<std::fixed<<std::setprecision(2);

    ss<<"Simulation duration: "<<simulation_info.simulation_duration;
    ss<<" [s]";
    GuiObject::text(ss.str());
    ss.str("");

    ss<<"Gravitational acceleration: "<<simulation_info.gravitational_acceleration;
    ss<<" [m/s^2]";
    GuiObject::text(ss.str());
    ss.str("");
    
    ss<<"Vehicles number: "<<simulation_info.vehicle_number;
    GuiObject::text(ss.str());
    ss.str("");

    ss<<"Track pos: "<<simulation_info.track_info.position;
    ss<<" [m]";
    GuiObject::text(ss.str());
    ss.str("");
    
    GuiObject::text("");

    for(const auto& vehicle_info:simulation_info.vehicles_info)
    {
      if (GuiObject::collapsingHeader("Vehicle")){

        ss<<"Vehicle chassis position: "<<vehicle_info.chassis_position;
        ss<<" [m]";
        GuiObject::text(ss.str());
        ss.str("");

        ss<<"Vehicle mass: "<<vehicle_info.mass;
        ss<<" [kg]";
        GuiObject::text(ss.str());
        ss.str("");

        ss<<"Vehicle speed: "<<vehicle_info.speed;
        ss<<" [m/s]";
        GuiObject::text(ss.str());
        ss.str("");

        ss<<"COM position in CS: "<<vehicle_info.center_of_mass_cs;
        ss<<" [m]";
        GuiObject::text(ss.str());
        ss.str("");

        GuiObject::text("Front left wheel");

        ss<<"\tPosition: "<<vehicle_info.front_left.position;
        ss<<" [m]";
        GuiObject::text(ss.str());
        ss.str("");

        GuiObject::text("Front right wheel");

        ss<<"\tPosition: "<<vehicle_info.front_right.position;
        ss<<" [m]";
        GuiObject::text(ss.str());
        ss.str("");

        GuiObject::text("Rear left wheel");

        ss<<"\tPosition: "<<vehicle_info.rear_left.position;
        ss<<" [m]";
        GuiObject::text(ss.str());
        ss.str("");

        GuiObject::text("Rear right wheel");

        ss<<"\tPosition: "<<vehicle_info.rear_right.position;
        ss<<" [m]";
        GuiObject::text(ss.str());
        ss.str("");
      }

    }
  }
}
