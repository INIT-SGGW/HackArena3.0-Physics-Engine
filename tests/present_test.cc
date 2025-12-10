#include "glm/ext/matrix_transform.hpp"

#include "glm/matrix.hpp"
#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/cube.hh"
#include "piksel/camera.hh"

#include "piksel/config.hh"

#include "boink/components/car_model.h"
#include "boink/world.h"
#include "boink/components/transform.h"
#include <Eigen/Core>

using namespace piksel;
using namespace Eigen;
using namespace boink;

int main(int argc, char **argv)
{
  constexpr float cam_speed=10.f;

  Window wnd("Test",1600,900);

  Camera cam({0.f,10.f,10.f},{0.f,0.f,0.f});
  Graphics gfx(wnd, cam);

  Cube car({1.f,0.5f,3.f,ASSET_PATH"/container.jpg",0});
  Cube ground({150.5f,0.5f,110.f,ASSET_PATH"/grass.png",1});

  gfx.AddCube(car);
  gfx.AddCube(ground);

  boink::CarModel car_model;
  car_model.front_left_wheel=Vector3d(-1.0,0.0,2.0);
  car_model.front_right_wheel=Vector3d(1.0,0.0,2.0);
  car_model.rear_left_wheel=Vector3d(-1.0,0.0,-2.0);
  car_model.rear_right_wheel=Vector3d(1.0,0.0,-2.0);
  car_model.max_steer_angle_deg=30;

  World world(car_model);
  Entity::ID id=world.car_manager.addCar();

  world.start(0.0);

  CarInput input;
  input.brake=0.0;
  input.steer_angle=0.0;
  input.throttle=0.0;
  world.car_manager.updateCar(id,input);
  const double simul_step=0.01;

  float last=glfwGetTime();
  float simul_last=glfwGetTime();
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

    auto [trans, input]=
      world.car_manager.getCarComponents<boink::Transform,CarInput>(id);

    if(wnd.getKey(GLFW_KEY_E)==Window::KeyState::Press){
      input.steer_angle+=0.1f;
    }
    if(wnd.getKey(GLFW_KEY_Q)==Window::KeyState::Press){
      input.steer_angle-=0.1f;
    }

    if(wnd.getKey(GLFW_KEY_R)==Window::KeyState::Press){
      input.throttle+=0.6f;
    }
    else{
      input.throttle=0;
    }
    if(wnd.getKey(GLFW_KEY_T)==Window::KeyState::Press){
      input.brake+=0.5f;
    }
    else{
      input.brake=0;
    }

    Window::MousePos mouse_pos=wnd.getMousePos();
    cam.rotateYaw((prev_mouse_pos.x-mouse_pos.x)*dt/3.f);
    prev_mouse_pos.x=mouse_pos.x;
    cam.rotatePitch((prev_mouse_pos.y-mouse_pos.y)*dt/3.f);
    prev_mouse_pos.y=mouse_pos.y;


    float simul_right_now=glfwGetTime();
    if(simul_right_now-simul_last>simul_step){
      simul_last=simul_right_now;
      
      world.car_manager.updateCar(id,input);
      world.update(simul_step);
    }
    
    const auto& pos=trans.position;
    const auto& rot=trans.rotation;

    glm::mat4 rotate_mat(rot(0,0),rot(1,0),rot(2,0),0.f,
                          rot(0,1),rot(1,1),rot(2,1),0.f,
                          rot(0,2),rot(1,2),rot(2,2),0.f,
                          0.f,0.f,0.f,1.f);


    car.translate=glm::translate(glm::mat4(1.f),{pos.x(),pos.y(),pos.z()});
    car.rotate=glm::transpose(rotate_mat);
    
    ground.translate=glm::translate(glm::mat4(1.f),{0.f,-2.00f,0.0f});
    ground.rotate=glm::mat4(1.f);;

    gfx.Render();
    wnd.update();
  }

  return 0;
}

