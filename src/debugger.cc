#include "boink/debugger/debugger.h"

#include <piksel/gui_object.hh>
#include <piksel/window.hh>

#include "boink/utility.h"
#include "boink/debugger/camera_controller.h"

#include <algorithm>

namespace boink
{
  Debugger::Debugger(
      std::string_view title,
      const btVector3& camera_position,
      const btVector3& camera_target)
    :
      wnd_(title.data()),
      cam_(bt2glm(camera_position),bt2glm(camera_target)),
      renderer_(wnd_,cam_),
      gui_manager_(wnd_.getGLFWPointer()),
      gui_(std::make_shared<DebuggerGui>()),
      default_controller_(std::make_shared<CameraController>()),
      fps_(0.f)
  {
    cam_.setMovementSpeed(5.f);
    cam_.setRotationSpeed(0.3f);
    mouse_speed_=cam_.getRotationSpeed();
    cam_speed_=cam_.getMovementSpeed();

    gui_->camera_speed=&cam_speed_;
    gui_->mouse_speed=&mouse_speed_;
    gui_->fps=&fps_;
    gui_->selected_controller=&selected_controller_;

    gui_manager_.addObject(gui_);
  }

  void Debugger::update()
  {
    static double prev=wnd_.getTime();
    double now=wnd_.getTime();
    float dt=(float)(now-prev);
    prev=now;

    cam_.setMovementSpeed(cam_speed_);
    cam_.setRotationSpeed(mouse_speed_);

    std::vector<std::pair<Simulator::ID,std::string>> con_pair;
    con_pair.reserve(controllers_.size());
    for(const auto& p : controllers_)
      con_pair.emplace_back(p.first,std::string("Vehicle id="+std::to_string(p.first)));
    gui_->controller_ids=std::move(con_pair);

    this->calculateFramerate(dt);

    this->handleWindowClose();
    this->updateController(dt);

    renderer_.render();
    gui_manager_.render();
    wnd_.update();
  }

  btScalar Debugger::getTime() const
  {
    return (btScalar)wnd_.getTime();
  }

  void Debugger::addGui(std::shared_ptr<piksel::GuiObject> gui_object)
  {
    gui_manager_.addObject(gui_object);
  }

  void Debugger::setControllers(
      std::vector<
        std::pair<Simulator::ID,std::shared_ptr<Controller>>> controllers)
  {
    controllers_=controllers;
  }

  bool Debugger::shouldClose() const
  {
    return !(bool)wnd_;
  }

  void Debugger::handleWindowClose()
  {
    if(wnd_.getKey(GLFW_KEY_ESCAPE)==piksel::Window::KeyState::Press)
      wnd_.close();
  }

  void Debugger::updateController(float dt)
  {
    if(selected_controller_==-1 &&
        wnd_.getKey(GLFW_KEY_LEFT_SHIFT)!=piksel::Window::KeyState::Press)
    {
      gui_manager_.ignoreInput();
      default_controller_->update(wnd_,cam_,dt);
    }
    else
    {
      gui_manager_.ignoreInput(false);
      wnd_.setCursor();
      default_controller_->updateMouse(wnd_);

      if(selected_controller_==-1)
        return;

      int selected=selected_controller_;
      auto it=std::find_if(controllers_.begin(),controllers_.end(),
          [=](const auto& p)
          {
            return p.first==(Simulator::ID)selected;
          });
      assert(it!=controllers_.end());

      it->second->update(wnd_,cam_,dt);
    }
  }
}
