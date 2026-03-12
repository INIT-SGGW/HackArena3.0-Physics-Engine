#pragma once

#include <LinearMath/btIDebugDraw.h>

#include <memory>
#include <piksel/IDrawable.hh>
#include <piksel/graphics.hh>
#include <piksel/window.hh>

namespace boink
{
  class Renderer : public btIDebugDraw
  {
  public:
    Renderer(
        piksel::Window& wnd, 
        const piksel::Camera& cam_,
        const piksel::Color& background=piksel::Color::Black);

    void render();
    void drawFrameOrigin();
    
    void setDebugMode(int mode) override { debug_mode_ = mode; }
    int getDebugMode() const override { return debug_mode_; }
    void addDrawable(std::shared_ptr<const piksel::IDrawable> drawable);

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
    void clearLines() override;

    void reportErrorWarning(const char*) override {}
    void draw3dText(const btVector3&, const char*) override {}

    int getDefaultDebugMode() const {return DBG_DrawWireframe;}
  private:
    static const std::string_view s_kSrcVertexShader_;
    static const std::string_view s_kSrcFragShader_;
  private:
    piksel::Graphics gfx_;
    int debug_mode_=DBG_DrawWireframe;
  };
}
