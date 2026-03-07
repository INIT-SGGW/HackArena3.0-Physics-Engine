#include "boink/gui/track_gui.h"

#include <imgui.h>

#include "boink/gui/surface_info_gui.h"
#include "boink/simulators/track/track.h"

namespace boink
{
  TrackGui::TrackGui(Track* p_track)
    :p_track_(p_track)
  {
    for(auto& p : p_track_->surface_infos_)
      surface_guis_.emplace(p.first,SurfaceInfoGui(&p.second));
  }

  void TrackGui::draw()
  {
    if(p_track_==nullptr)
      return;

    ImGui::Text("File: %s",p_track_->getFilename().data());

    btVector3 pos=p_track_->getWorldTransform().getOrigin();
    ImGui::Text("Position: (%.2f,%.2f,%.2f)",
        pos.getX(),pos.getY(),pos.getZ());

    ImGui::Checkbox("Enable track data vectors draw",
        &p_track_->enable_track_data_vec_draw_);

    for(auto& p:surface_guis_)
    {
      ImGui::PushID((int)p.first);
      if(ImGui::CollapsingHeader(p.second.getTitle().data()))
        p.second.draw();
      ImGui::PopID();
    }
  }
}
