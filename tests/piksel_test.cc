#include "glm/ext/matrix_transform.hpp"
#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/camera.hh"
#include "piksel/color.hh"
#include "piksel/model.hh"
#include "piksel/config.hh"

#include <glm/gtc/matrix_transform.hpp>

#include <memory>

using namespace piksel;

int main()
{
  constexpr float cam_speed=10.f; 
  Window wnd("Model load test", 1280,720);
  Camera cam({0.f,0.f,10.f},{0.f,0.f,0.f});
  Graphics gfx(
      wnd,cam,SHADERS_PATH"/single_color.vert",SHADERS_PATH"/single_color.frag");
  gfx.setBackground(Color::Blue);

  auto car = std::make_shared<Model>(ASSETS_PATH"/models/F1_bolid.glb");
  auto track = std::make_shared<Model>(ASSETS_PATH"/models/tor.glb");

  car->color=Color::Green;
  track->color=Color::White;

  double scale_factor=0.25;
  track->scale=glm::scale(track->scale,glm::vec3(scale_factor));

  gfx.addObject(car);
  gfx.addObject(track);
  
  float last=glfwGetTime();
  Window::MousePos prev_mouse_pos=wnd.getMousePos();
  while(wnd)
  {
    float right_now=glfwGetTime();
    float dt=right_now-last;
    last=right_now;

    if(wnd.getKey(GLFW_KEY_ESCAPE)==Window::KeyState::Press)
      wnd.close();
    
    if(wnd.getKey(GLFW_KEY_W)==Window::KeyState::Press){
      cam.moveLongitudinal(cam_speed*dt);
    }
    if(wnd.getKey(GLFW_KEY_S)==Window::KeyState::Press){
      cam.moveLongitudinal(-cam_speed*dt);
    }

    if(wnd.getKey(GLFW_KEY_A)==Window::KeyState::Press){
      cam.moveLateral(-cam_speed*dt);
    }
    if(wnd.getKey(GLFW_KEY_D)==Window::KeyState::Press){
      cam.moveLateral(cam_speed*dt);
    }

    Window::MousePos mouse_pos=wnd.getMousePos();
    cam.rotateYaw((prev_mouse_pos.x-mouse_pos.x)*dt/3.f);
    prev_mouse_pos.x=mouse_pos.x;
    cam.rotatePitch((prev_mouse_pos.y-mouse_pos.y)*dt/3.f);
    prev_mouse_pos.y=mouse_pos.y;

    gfx.render();
    wnd.update();
  }
  return 0;
}
