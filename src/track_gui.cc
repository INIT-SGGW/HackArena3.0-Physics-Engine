#include "boink/gui/track_gui.h"

#include <imgui.h>

namespace boink
{
  void TrackGui::draw()
  {
    ImGui::Text("File: %s",filename.data());
    ImGui::Text("Position: (%.2f,%.2f,%.2f)",pos.getX(),pos.getY(),pos.getZ());
  }
}
