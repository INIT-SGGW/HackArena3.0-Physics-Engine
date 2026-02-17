#pragma once

#include <piksel/gui_object.hh>

#include "boink/simulators/simulator.h"

#include <memory>

namespace boink
{
  class SimulationGui : public piksel::GuiObject
  {
  public:
    virtual std::string_view getTitle() const override{return "Simulation";}
    virtual void draw() override;

    void addSimulatorGui(Simulator::ID id,std::shared_ptr<piksel::GuiObject> gui);
    void removeSimulatorGui(Simulator::ID id);
  public:
    bool freeze=true;
    float duration;
    float gravity_acc;
    size_t num_simulators;
  private:
    std::vector<std::pair<Simulator::ID,std::shared_ptr<piksel::GuiObject>>> guis_;
  };
}
