#include "boink/debugger/debugger.h"

#include "boink/utility.h"

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
      fps_(0.f)
  {
    cam_.setMovementSpeed(5.f);
    cam_.setRotationSpeed(0.3f);
  }

  void Debugger::update()
  {
    static double prev=wnd_.getTime();
    double now=wnd_.getTime();
    float dt=(float)(now-prev);
    prev=now;

    this->calculateFramerate(dt);

    this->handleWindowClose();
    this->handleCameraMovement(dt);

    renderer_.render();
    wnd_.update();
  }

  btScalar Debugger::getTime() const
  {
    return (btScalar)wnd_.getTime();
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

  void Debugger::handleCameraMovement(float dt)
  {
    static piksel::Window::MousePos prev_mouse_pos=wnd_.getMousePos();

    if(wnd_.getKey(GLFW_KEY_W)==piksel::Window::KeyState::Press){
      cam_.moveLongitudinal(dt);
    }
    if(wnd_.getKey(GLFW_KEY_S)==piksel::Window::KeyState::Press){
      cam_.moveLongitudinal(-dt);
    }

    if(wnd_.getKey(GLFW_KEY_A)==piksel::Window::KeyState::Press){
      cam_.moveLateral(-dt);
    }
    if(wnd_.getKey(GLFW_KEY_D)==piksel::Window::KeyState::Press){
      cam_.moveLateral(dt);
    }

    piksel::Window::MousePos mouse_pos=wnd_.getMousePos();
    wnd_.setCursor(false);
    cam_.rotateYaw((float)(prev_mouse_pos.x-mouse_pos.x)*dt);
    cam_.rotatePitch((float)(prev_mouse_pos.y-mouse_pos.y)*dt);

    prev_mouse_pos.x=mouse_pos.x;
    prev_mouse_pos.y=mouse_pos.y;
  }
}
