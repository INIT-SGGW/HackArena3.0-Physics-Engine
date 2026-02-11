#include "boink/debug_drawer.h"

#include "boink/version.h"

#include <LinearMath/btIDebugDraw.h>
#include <memory>
#include <piksel/IDrawable.hh>
#include <piksel/color.hh>
#include <piksel/object.hh>
#include <piksel/shader.hh>
#include <piksel/window.hh>
#include <vector>

#include "boink/utils/utility.h"

namespace boink
{
  DebugDrawer::DebugDrawer()
    :
      wnd_(s_kWindowName_),
      cam_({0.f,0.f,10.f},{0.f,0.f,0.f}),
      gfx_(wnd_,cam_,
          piksel::Shader(
              piksel::Shader::CompileShader(
                s_kSrcVertexShader_,piksel::Shader::ShaderType::VertexType),
              piksel::Shader::CompileShader(
                s_kSrcFragShader_,piksel::Shader::ShaderType::FragmentType)
              )),
      gui_manager_(wnd_.getGLFWPointer())
  {
    gfx_.setBackground(s_kBackgroundColor_);
    gui_manager_.addObject(info_);
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
    static float dt=(float)getDeltaTime();
    dt=(float)getDeltaTime();

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
    if(wnd_.getKey(GLFW_KEY_LEFT_SHIFT)==piksel::Window::KeyState::Press){
      wnd_.setCursor();
      gui_manager_.ignoreInput(false);
    }
    else
    {
      if(info_->isCameraEnable())
      {
        gui_manager_.ignoreInput();
        wnd_.setCursor(false);
        cam_.rotateYaw((float)(prev_mouse_pos.x-mouse_pos.x)*dt*mouse_speed_);
        cam_.rotatePitch((float)(prev_mouse_pos.y-mouse_pos.y)*dt*mouse_speed_);
      }
    }
    prev_mouse_pos.x=mouse_pos.x;
    prev_mouse_pos.y=mouse_pos.y;

    mouse_speed_=info_->getMouseSpeed();
    cam_speed_=info_->getCameraSpeed();

    gfx_.clear();
    gfx_.render();
    gui_manager_.render();
    wnd_.update();
  }

  void DebugDrawer::setCamera(const btVector3& pos, const btVector3& target)
  {
    cam_.set(bt2glm(target),bt2glm(pos));
  }

  void DebugDrawer::addObject(std::shared_ptr<piksel::IDrawable> obj)
  {
    gfx_.addDrawable(obj);
  }

  double DebugDrawer::getTime() const
  {
    return glfwGetTime();
  }

  bool DebugDrawer::isSimulationToFreeze() const
  {
    return info_->isSimulationFreeze();
  }

  double DebugDrawer::getDeltaTime()
  {
    double time=glfwGetTime();
    double dt=time-prev_time_;
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

  void DebugDrawer::drawLines(
      const std::vector<btVector3> points,
      const btVector3& color,
      float elapsed_time)
  {
    assert(points.size()>=2);
    //for(size_t i=1;i<points.size();i++)
    if(elapsed_time>points.size())
      elapsed_time=(float)points.size();
    for(size_t i=1;i<elapsed_time;i++)
    {
      drawLine(points[i-1],points[i],color);
    }
  }

  piksel::Window::KeyState DebugDrawer::getKey(int glfw_key) const
  {
    return wnd_.getKey(glfw_key);
  }

  SimulationInfo& DebugDrawer::getSimulationInfo()
  {
    return info_->simulation_info;
  }

#ifdef NDEBUG
  const char* DebugDrawer::s_kWindowName_="Release Boink (" BOINK_VERSION ")";
#else
  const char* DebugDrawer::s_kWindowName_="Debug Boink (" BOINK_VERSION ")";
#endif
  const piksel::Color DebugDrawer::s_kBackgroundColor_= piksel::Color::Black;
#ifdef RASPBERRY_PI
  const std::string_view DebugDrawer::s_kSrcVertexShader_="#version 300 es\n\nlayout (location = 0) in vec3 aPos;\nlayout (location = 1) in vec2 aTexCord;\n\nout vec2 ourTexCord;\n\nuniform mat4 proj;\nuniform mat4 view;\nuniform mat4 trans;\n\nvoid main()\n{\n  ourTexCord=aTexCord;\n  gl_Position = proj*view*trans*vec4(aPos.xyz, 1.0f);\n};\n";
  const std::string_view DebugDrawer::s_kSrcFragShader_="#version 300 es\n\nprecision mediump float;\n\nin vec2 ourTexCord;\n\nout vec4 FragColor;\n\nuniform vec3 color;\n\nvoid main()\n{\n  FragColor=vec4(color.xyz,1.0);\n};\n";
#else
  const std::string_view DebugDrawer::s_kSrcVertexShader_="#version 330 core\n\nlayout (location = 0) in vec3 aPos;\nlayout (location = 1) in vec2 aTexCord;\n\nout vec2 ourTexCord;\n\nuniform mat4 proj;\nuniform mat4 view;\nuniform mat4 trans;\n\nvoid main()\n{\n  ourTexCord=aTexCord;\n  gl_Position = proj*view*trans*vec4(aPos.xyz, 1.0f);\n};\n";
  const std::string_view DebugDrawer::s_kSrcFragShader_="#version 330 core\n\nin vec2 ourTexCord;\n\nout vec4 FragColor;\n\nuniform vec3 color;\n\nvoid main()\n{\n  FragColor=vec4(color.xyz,1.0);\n};\n";
#endif
}
