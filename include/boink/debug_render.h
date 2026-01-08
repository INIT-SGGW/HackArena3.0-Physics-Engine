#pragma once

#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/camera.hh"

#include <LinearMath/btIDebugDraw.h>
#include <glm/glm.hpp>

#include <piksel/object.hh>

#include <memory>
#include <iostream>

namespace boink
{
  class DebugRender : public btIDebugDraw
  {
  public:
    struct Line
    {
      glm::vec3 from;
      glm::vec3 to;
      glm::vec3 color;
    };
  public:
    DebugRender();
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

    void draw3dText(const btVector3&, const char*) override {}
    void setDebugMode(int mode) override { debug_mode_ = mode; }
    int getDebugMode() const override { return debug_mode_; }

    /////// Mine /////

    explicit operator bool() const;

    void update(float dt);
    void setCameraSpeed(float speed);
    void addObject(std::shared_ptr<piksel::Object> obj);
    void removeObject(std::shared_ptr<piksel::Object> obj);
    float getDeltaTime() const;
  private:
    piksel::Window wnd_;
    piksel::Camera cam_;
    piksel::Graphics gfx_;

    int debug_mode_;

    float cam_speed_=5.f;
    float mouse_speed_=0.3f;

    mutable float prev_time_;
  };
}
