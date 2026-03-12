#include "boink/gui/debugger_gui.h"

#include <imgui.h>

#include <algorithm>

namespace boink
{
  DebuggerGui::DebuggerGui(Debugger* p_debug)
    :p_debug(p_debug)
  {

  }

  void DebuggerGui::draw()
  {
    ImGui::Text("FPS: %.2f",p_debug->fps_);
    ImGui::SliderFloat("Mouse speed",&p_debug->mouse_speed_,0.f,1.f);
    ImGui::SliderFloat("Camera speed",&p_debug->cam_speed_,0.f,100.f);
    ImGui::Checkbox("Enable bullet draw",&bullet_draw_);
    
    if(bullet_draw_)
      p_debug->renderer_.setDebugMode(p_debug->renderer_.getDefaultDebugMode());
    else
      p_debug->renderer_.setDebugMode(0);

    static size_t selected=0;
    static size_t selected2=0;
    const char* camera_option="Free camera";

    std::sort(controller_ids.begin(),controller_ids.end());
    controller_ids.insert(controller_ids.begin(),{-1,camera_option});

    const char* preview=controller_ids[selected].second.c_str();
    const char* preview2=controller_ids[selected2].second.c_str();

    if(ImGui::BeginCombo("Contoller",preview))
    {
      for(size_t i=0;i<controller_ids.size();i++)
      {
        bool is_selected=(selected==i);
        if (ImGui::Selectable(controller_ids[i].second.c_str(), is_selected))
          selected = i;

        if (is_selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }
    if(controller_ids[selected].second==camera_option)
      p_debug->selected_controller_=-1;
    else
      p_debug->selected_controller_=(int)controller_ids[selected].first;

    if(ImGui::BeginCombo("Contoller2",preview2))
    {
      for(size_t i=0;i<controller_ids.size();i++)
      {
        bool is_selected=(selected2==i);
        if (ImGui::Selectable(controller_ids[i].second.c_str(), is_selected))
          selected2 = i;

        if (is_selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }

    if(controller_ids[selected2].second==camera_option)
      p_debug->selected_controller_2_=-1;
    else
      p_debug->selected_controller_2_=(int)controller_ids[selected2].first;
  }
}
