#include "boink/gui/debugger_gui.h"

#include <imgui.h>

#include <algorithm>

namespace boink
{
  void DebuggerGui::draw()
  {
    ImGui::Text("FPS: %.2f",fps);
    ImGui::SliderFloat("Mouse speed",&mouse_speed,0.f,1.f);
    ImGui::SliderFloat("Camera speed",&camera_speed,0.f,100.f);

    static size_t selected=0;
    const char* camera_option="Free camera";

    std::sort(controller_ids.begin(),controller_ids.end());
    controller_ids.insert(controller_ids.begin(),{-1,camera_option});

    const char* preview=controller_ids[selected].second.c_str();

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
      selected_controller=-1;
    else
      selected_controller=controller_ids[selected].first;
  }
}
