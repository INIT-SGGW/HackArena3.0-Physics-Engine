// clang-format off
#include "boink/gui/surface_info_gui.h"

#include <imgui.h>

namespace boink
{
  SurfaceInfoGui::SurfaceInfoGui(Ground::SurfaceInfo* p_info)
    :p_info_(p_info),title_(Ground::toString(p_info->type))
  {
    title_=std::string("Surface: ")+title_;
  }

  void SurfaceInfoGui::draw()
  {
    ImGui::SliderFloat(
        "Grip coefficient",&p_info_->grip_coeff,0.f,2.f);
    ImGui::SliderFloat(
        "Rolling resistance",&p_info_->rolling_resistance,0.f,2.f);
    ImGui::Text("Wetness: %.2f",p_info_->wetness);
  }
}
