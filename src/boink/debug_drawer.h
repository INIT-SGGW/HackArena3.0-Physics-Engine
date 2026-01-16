#pragma once

#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/camera.hh"

#include <LinearMath/btIDebugDraw.h>
#include <glm/glm.hpp>

#include <piksel/IDrawable.hh>
#include <piksel/object.hh>

#include <memory>
#include <iostream>

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
    void setCameraSpeed(float speed);
    void addObject(std::shared_ptr<piksel::IDrawable> obj);
    //void removeObject(std::shared_ptr<piksel::IDrawable> obj);
    double getTime() const;

    void drawFrameOrigin();

    piksel::Window::KeyState getKey(int glfw_key) const;
  private:
    double getDeltaTime();
  private:
    static std::string_view src_code_vertex_sh;
    static std::string_view src_code_frag_sh;
  private:
    piksel::Window wnd_;
    piksel::Camera cam_;
    piksel::Graphics gfx_;

    int debug_mode_;

    float cam_speed_=5.f;
    float mouse_speed_=0.3f;

    double prev_time_;
  };
}
