#include "boink/debug_drawer.h"

#include "piksel/config.hh"
#include <LinearMath/btIDebugDraw.h>
#include <memory>
#include <piksel/IDrawable.hh>
#include <piksel/color.hh>
#include <piksel/object.hh>
#include <piksel/window.hh>

#include "boink/utils/utility.h"

namespace boink
{
  DebugDrawer::DebugDrawer()
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

  void DebugDrawer::drawLine(
        const btVector3& from,
        const btVector3& to,
        const btVector3& color)
  {
    gfx_.drawLine(piksel::Line{bt2glm(from),bt2glm(to),bt2glm(color)});
  }

  void DebugDrawer::drawContactPoint(
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

  void DebugDrawer::clearLines()
  {
    gfx_.clearLines();
  }

  void DebugDrawer::drawPoint(
      const btVector3& point, 
      const btVector3& color)
  {
    drawLine(
        point,
        point+btVector3{5.f,0.f,0.f},
        color);
    drawLine(
        point,
        point+btVector3{0.f,5.f,0.f},
        color);
  }

  DebugDrawer::operator bool() const
  {
    return (bool)wnd_;
  }

  void DebugDrawer::update()
  {
    static double dt=getDeltaTime();
    dt=getDeltaTime();

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

  void DebugDrawer::setCameraSpeed(float speed)
  {
    cam_speed_=speed;
  }

  void DebugDrawer::addObject(std::shared_ptr<piksel::IDrawable> obj)
  {
    gfx_.addDrawable(obj);
  }

  float DebugDrawer::getTime() const
  {
    return glfwGetTime();
  }

  float DebugDrawer::getDeltaTime()
  {
    float time=glfwGetTime();
    float dt=time-prev_time_;
    prev_time_=time;

    return dt;
  }

  void DebugDrawer::drawFrameOrigin()
  {
    drawLine(
        {0.f,0.f,0.f},
        {10.f,0.f,0.f},
        {1.f,0.f,0.f});
    drawLine(
        {0.f,0.f,0.f},
        {0.f,10.f,0.f},
        {0.f,1.f,0.f});
    drawLine(
        {0.f,0.f,0.f},
        {0.f,0.f,10.f},
        {0.f,0.f,1.f});
  }

  piksel::Window::KeyState DebugDrawer::getKey(int glfw_key) const
  {
    return wnd_.getKey(glfw_key);
  }
}
