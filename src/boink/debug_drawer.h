#pragma once

#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/camera.hh"
#include "piksel/gui_manager.hh"

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <LinearMath/btIDebugDraw.h>
#include <glm/glm.hpp>

#include <piksel/IDrawable.hh>
#include <piksel/color.hh>
#include <piksel/object.hh>

#include "boink/debug_info_gui.h"

#include <memory>
#include <iostream>
#include <vector>

namespace boink
{
  class DebugDrawer : public btIDebugDraw
  {
  public:
    struct Line
    {
      glm::vec3 from;
      glm::vec3 to;
      glm::vec3 color;
    };
  public:
    DebugDrawer();
    void drawLine(
        const btVector3& from,
        const btVector3& to,
        const btVector3& color) override;
    void drawContactPoint(
        const btVector3& pointOnB,
        const btVector3& normalOnB,
        btScalar,
        int,
        const btVector3& color
    ) override;

    void reportErrorWarning(const char* warningString) override
    {
        std::cout << "Bullet: " << warningString << std::endl;
    }

    void clearLines() override;

    void draw3dText(const btVector3&, const char*) override {}
    void setDebugMode(int mode) override { debug_mode_ = mode; }
    int getDebugMode() const override { return debug_mode_; }

    /////// Mine /////

    void drawPoint(
        const btVector3& point, 
        const btVector3& color);

    explicit operator bool() const;

    void update();

    void setCamera(const btVector3& pos, const btVector3& target);
    void addObject(std::shared_ptr<piksel::IDrawable> obj);
    double getTime() const;
    bool isSimulationToFreeze() const;

    void drawFrameOrigin();
    void drawLines(
      const std::vector<btVector3> points,
      const btVector3& color,
      float elapsed_time);

    piksel::Window::KeyState getKey(int glfw_key) const;
    SimulationInfo& getSimulationInfo();

  private:
    double getDeltaTime();
  private:
    static const std::string_view s_kSrcVertexShader_;
    static const std::string_view s_kSrcFragShader_;

    static const piksel::Color s_kBackgroundColor_;
    static const char* s_kWindowName_;
  private:
    piksel::Window wnd_;
    piksel::Camera cam_;
    piksel::Graphics gfx_;
    piksel::GuiManager gui_manager_;

    int debug_mode_=btIDebugDraw::DebugDrawModes::DBG_DrawWireframe;

    float cam_speed_=5.f;
    float mouse_speed_=0.3f;

    double prev_time_=glfwGetTime();

    std::shared_ptr<DebugInfoGui> info_=std::make_shared<DebugInfoGui>();
  };

}
