#include "boink/debugger/renderer.h"

#include <piksel/graphics.hh>

#include "boink/utility.h"

namespace boink
{
  Renderer::Renderer(
      piksel::Window& wnd, 
      const piksel::Camera& cam,
      const piksel::Color& background)
    :
      gfx_(wnd,cam,
          piksel::Shader(
            piksel::Shader::CompileShader(
              s_kSrcVertexShader_,piksel::Shader::ShaderType::VertexType),
            piksel::Shader::CompileShader(
              s_kSrcFragShader_,piksel::Shader::ShaderType::FragmentType)
            ))
  {
    gfx_.setBackground(background);
  }

  void Renderer::render()
  {
    gfx_.clear();
    gfx_.render();
  }

  void Renderer::drawFrameOrigin()
  {
    this->drawLine(
        {0.f,0.f,0.f},
        {10.f,0.f,0.f},
        {1.f,0.f,0.f});
    this->drawLine(
        {0.f,0.f,0.f},
        {0.f,10.f,0.f},
        {0.f,1.f,0.f});
    this->drawLine(
        {0.f,0.f,0.f},
        {0.f,0.f,10.f},
        {0.f,0.f,1.f});
  }

  void Renderer::addDrawable(
      std::shared_ptr<const piksel::IDrawable> drawable)
  {
    gfx_.addDrawable(drawable);
  }

  void Renderer::drawLine(
        const btVector3& from,
        const btVector3& to,
        const btVector3& color)
  {
    gfx_.drawLine(piksel::Line{math::bt2glm(from),math::bt2glm(to),math::bt2glm(color)});
  }

  void Renderer::drawContactPoint(
      const btVector3& pointOnB,
      const btVector3& normalOnB,
      btScalar,
      int,
      const btVector3& color)
  {
    this->drawLine(
      pointOnB,
      pointOnB + normalOnB * 0.2f,
      color
    );
  }

  void Renderer::clearLines()
  {
    gfx_.clearLines();
  }

#ifdef RASPBERRY_PI
  const std::string_view Renderer::s_kSrcVertexShader_="#version 300 es\n\nlayout (location = 0) in vec3 aPos;\nlayout (location = 1) in vec2 aTexCord;\n\nout vec2 ourTexCord;\n\nuniform mat4 proj;\nuniform mat4 view;\nuniform mat4 trans;\n\nvoid main()\n{\n  ourTexCord=aTexCord;\n  gl_Position = proj*view*trans*vec4(aPos.xyz, 1.0f);\n}\n";
  const std::string_view Renderer::s_kSrcFragShader_="#version 300 es\n\nprecision mediump float;\n\nin vec2 ourTexCord;\n\nout vec4 FragColor;\n\nuniform vec3 color;\n\nvoid main()\n{\n  FragColor=vec4(color.xyz,1.0);\n}\n";
#else
const std::string_view Renderer::s_kSrcVertexShader_ =
"#version 330 core\n"
"layout(location = 0) in vec3 aPos;\n"
"layout(location = 1) in vec2 aTexCord;\n"
"out vec2 ourTexCord;\n"
"uniform mat4 proj;\n"
"uniform mat4 view;\n"
"uniform mat4 trans;\n"
"void main() {\n"
"  ourTexCord = aTexCord;\n"
"  gl_Position = proj * view * trans * vec4(aPos, 1.0);\n"
"}\n";

const std::string_view Renderer::s_kSrcFragShader_ =
"#version 330 core\n"
"in vec2 ourTexCord;\n"
"out vec4 FragColor;\n"
"uniform vec3 color;\n"
"void main() {\n"
"  vec4 c = vec4(color, 1.0);\n"
"  vec4 t = vec4(ourTexCord, 0.0, 0.0);\n"
"  FragColor = c + t;\n"
"}\n";
#endif
}
