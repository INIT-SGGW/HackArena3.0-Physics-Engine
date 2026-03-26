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

    void addSimulatorGui(
        Simulator::ID id,std::shared_ptr<piksel::GuiObject> gui);
    void removeSimulatorGui(Simulator::ID id);
  public:
    bool* freeze=nullptr;
    const float* duration=nullptr;
    float gravity_acc=10.f;
    size_t num_simulators=0;
  private:
    std::vector<std::pair<Simulator::ID,std::shared_ptr<piksel::GuiObject>>> 
      guis_;
  };
}
