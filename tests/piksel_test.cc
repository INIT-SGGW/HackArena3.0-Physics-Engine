#include "glm/ext/matrix_transform.hpp"
#include "glm/trigonometric.hpp"

#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/cube.hh"
#include "piksel/camera.hh"

#include "piksel/config.hh"

using namespace piksel;

int main(int argc, char **argv)
{
  constexpr float cam_speed=10.f;

  Window wnd("Test",1600,900);

  Camera cam({10.f,0.f,0.f},{0.f,0.f,0.f});
  Graphics gfx(wnd, cam);
  //glfwSwapInterval(0);

  Cube cube({1.f,1.0f,1.f,ASSET_PATH"/container.jpg",0});
  Cube cube2({5.5f,1.5f,1.f,ASSET_PATH"/container.jpg",0});
  Cube cube3({15.5f,1.5f,11.f,ASSET_PATH"/container.jpg",0});

  gfx.AddCube(cube);
  gfx.AddCube(cube2);
  gfx.AddCube(cube3);

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

    cube.translate=glm::translate(glm::mat4(1.f),{0.f,0.00f,0.0f});
    cube.rotate=glm::rotate(cube.rotate,glm::radians(360.f*dt/2),
        {1.f,0.f,1.f});
   
    cube2.translate=glm::translate(glm::mat4(1.f),{0.f,0.00f,3.0f});
    cube2.rotate=glm::rotate(cube.rotate,glm::radians(360.f*dt/2),
        {1.f,0.f,1.f});
    
    cube3.translate=glm::translate(glm::mat4(1.f),{0.f,0.00f,0.0f});
    cube3.rotate=glm::mat4(1.f);;

    gfx.Render();
    wnd.update();
  }

  return 0;
}

