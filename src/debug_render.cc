#include "boink/debug_render.h"

#include "piksel/config.hh"
#include <LinearMath/btIDebugDraw.h>
#include <memory>
#include <piksel/color.hh>
#include <piksel/object.hh>

namespace boink
{
  inline glm::vec3 bt2glm(const btVector3& vec)
  {
    return glm::vec3(vec.x(),vec.y(),vec.z());
  }

  inline btVector3 glm2bt(const glm::vec3& vec)
  {
    return btVector3(vec.x,vec.y,vec.z);
  }

  DebugRender::DebugRender()
    :wnd_("Debug Window",1280,720),
    cam_({0.f,0.f,10.f},{0.f,0.f,0.f}),
    gfx_(wnd_,cam_,
        PIKSEL_SHADERS_PATH"/single_color.vert",
        PIKSEL_SHADERS_PATH"/single_color.frag"),
    debug_mode_(btIDebugDraw::DebugDrawModes::DBG_DrawWireframe),
    prev_time_(glfwGetTime())
  {
    gfx_.setBackground(piksel::Color::Black);
  }

  void DebugRender::drawLine(
        const btVector3& from,
        const btVector3& to,
        const btVector3& color)
  {
    gfx_.drawLine(piksel::Line{bt2glm(from),bt2glm(to),bt2glm(color)});
  }

  void DebugRender::drawContactPoint(
      const btVector3& pointOnB,
      const btVector3& normalOnB,
      btScalar,
      int,
      const btVector3& color)
  {
    drawLine(
      pointOnB,
      pointOnB + normalOnB * 0.2f,
      color
    );
  }

  DebugRender::operator bool() const
  {
    return (bool)wnd_;
  }

  void DebugRender::update(float dt)
  {
    static piksel::Window::MousePos prev_mouse_pos=wnd_.getMousePos();

    if(wnd_.getKey(GLFW_KEY_ESCAPE)==piksel::Window::KeyState::Press)
      wnd_.close();
    
    if(wnd_.getKey(GLFW_KEY_W)==piksel::Window::KeyState::Press){
      cam_.moveLongitudinal(cam_speed_*dt);
    }
    if(wnd_.getKey(GLFW_KEY_S)==piksel::Window::KeyState::Press){
      cam_.moveLongitudinal(-cam_speed_*dt);
    }

    if(wnd_.getKey(GLFW_KEY_A)==piksel::Window::KeyState::Press){
      cam_.moveLateral(-cam_speed_*dt);
    }
    if(wnd_.getKey(GLFW_KEY_D)==piksel::Window::KeyState::Press){
      cam_.moveLateral(cam_speed_*dt);
    }

    piksel::Window::MousePos mouse_pos=wnd_.getMousePos();
    cam_.rotateYaw((prev_mouse_pos.x-mouse_pos.x)*dt*mouse_speed_);
    prev_mouse_pos.x=mouse_pos.x;
    cam_.rotatePitch((prev_mouse_pos.y-mouse_pos.y)*dt*mouse_speed_);
    prev_mouse_pos.y=mouse_pos.y;

    gfx_.render();
    wnd_.update();
  }

  void DebugRender::setCameraSpeed(float speed)
  {
    cam_speed_=speed;
  }

  void DebugRender::addObject(std::shared_ptr<piksel::Object> obj)
  {
    gfx_.addObject(obj);
  }

  float DebugRender::getDeltaTime() const
  {
    float time=glfwGetTime();
    float dt=time-prev_time_;
    prev_time_=time;

    return dt;
  }
}
