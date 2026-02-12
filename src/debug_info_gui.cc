#include "boink/debug_info_gui.h"

#include <imgui.h>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

namespace boink
{
  static inline std::ostream& operator<<(std::ostream& os, const btVector3& v)
  {
    os << '(' << v.x() << ", " << v.y() << ", " << v.z() << ')';
      return os;
  }
  
  void DebugInfoGui::draw()
  {

    GuiObject::text("");
    GuiObject::checkBox("Freeze simulation",&freeze_sim_);
    GuiObject::checkBox("Enable camera",&enable_camera_);
    GuiObject::slider("Mouse speed",0.f,1.f,&mouse_speed_);
    GuiObject::slider("Camera speed",0.f,100.f,&camera_speed_);
    GuiObject::text("");

    std::stringstream ss;
    ss<<std::fixed<<std::setprecision(2);

    ss<<"Debug framerate: "<<std::setw(6)<<fps_;
    ss<<" [fps]";
    GuiObject::text(ss.str());
    ss.str("");

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

    this->selectCar();

    GuiObject::text("");

    for (auto& [key,vehicle_info] : simulation_info.vehicles_info)
    {
      //auto& vehicle_info=simulation_info.vehicles_info[i];
      ImGui::PushID(key);
      ss<<"Vehicle ID: "<<key;
      if (GuiObject::collapsingHeader(ss.str())){
        ss.str("");

        GuiObject::slider(
            "Friction slip",0.f,10.f,&vehicle_info.friction_slip);
        GuiObject::slider(
            "Max suspension force",0.f,50000.f,&vehicle_info.max_suspension_force);
        GuiObject::slider(
            "Max suspension travel",0.f,20.f,&vehicle_info.max_suspension_travel_cm);
        GuiObject::slider(
            "Suspension compression",0.f,100.f,&vehicle_info.suspension_compression);
        GuiObject::slider(
            "Suspension damping",0.f,100.f,&vehicle_info.suspension_damping);
        GuiObject::slider(
            "Suspension stifness",0.f,100.f,&vehicle_info.suspension_stiffness);

        ss<<"Laps completed: "<<vehicle_info.laps_completed;
        GuiObject::text(ss.str());
        ss.str("");

        ss<<"Current lap coverage: "<<vehicle_info.curr_lap_coverage;
        ss<<" [m]";
        GuiObject::text(ss.str());
        ss.str("");

        ss<<"Vehicle chassis position: "<<vehicle_info.chassis_position.getOrigin();
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
      ImGui::PopID();

    }
  }

  void DebugInfoGui::selectCar()
  {
    static size_t selected=0;
    const char* camera_option="Free camera";

    const auto& vehicles_info=simulation_info.vehicles_info;
    std::vector<std::string> vehicles_ids;
    vehicles_ids.reserve(vehicles_info.size()+1);

    for(const auto& [key,_]:vehicles_info)
    {
      vehicles_ids.push_back(std::to_string(key));
    }
    std::sort(vehicles_ids.begin(),vehicles_ids.end());
    vehicles_ids.insert(vehicles_ids.begin(),camera_option);

    const char* preview=vehicles_ids[selected].c_str();

    if(ImGui::BeginCombo("Follow vehicle",preview))
    {
      for(size_t i=0;i<vehicles_ids.size();i++)
      {
        bool is_selected=(selected==i);
        if (ImGui::Selectable(vehicles_ids[i].c_str(), is_selected))
          selected = i;

        if (is_selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }
    if(vehicles_ids[selected]==camera_option)
    {
      selected_vehicle_id_=-1;
    }
    else
    {
      enable_camera_=false;
      selected_vehicle_id_=std::stoi(vehicles_ids[selected]);
    }
  }
}
