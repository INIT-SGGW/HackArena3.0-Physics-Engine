#include "boink/gui/ghost_gui.h"

#include <imgui.h>

namespace boink
{
  GhostGui::GhostGui(GhostMode* p_ghost)
    :p_ghost_(p_ghost)
  {}

  void GhostGui::draw()
  {
    ImGui::Text("Is sim on: %s",
        p_ghost_->is_sim_enabled_?"true":"false");
    ImGui::Text("Is in ghost mode: %s",
        p_ghost_->is_in_ghost_mode_?"true":"false");
    ImGui::Text("Enter timer curr: %f",
        p_ghost_->enter_timer_.getCurrent());
    ImGui::Text("Exit timer curr: %f",
        p_ghost_->exit_timer_.getCurrent());
    ImGui::Text("Force timer curr: %f",
        p_ghost_->force_timer_.getCurrent());
    ImGui::Text("Is active enter timer: %s",
        !p_ghost_->enter_timer_.hasFinised()?"true":"false");
    ImGui::Text("Is active exit timer: %s",
        !p_ghost_->exit_timer_.hasFinised()?"true":"false");
    ImGui::Text("Is active force timer: %s",
        !p_ghost_->force_timer_.hasFinised()?"true":"false");
  }
}
