#include <Eigen/Core>

#include "boink/components/car_model.h"
#include "boink/components/transform.h"
#include "boink/world.h"
#include "glm/ext/matrix_transform.hpp"
#include "glm/matrix.hpp"
#include "piksel/camera.hh"
#include "piksel/config.hh"
#include "piksel/graphics.hh"
#include "piksel/model.hh"
#include "piksel/window.hh"

#ifdef _WIN32
#define PROJECT_PATH "E:\\RepozytoriaGIT\\HackArena3.0-Physics-Engine"
#else
#define PROJECT_PATH "."
#endif

using namespace piksel;
using namespace Eigen;
using namespace boink;

int main(int argc, char** argv)
{
  constexpr float cam_speed = 10.f;

  Window wnd("Test", 1600, 900);

  Camera cam({0.f, 10.f, 10.f}, {0.f, 0.f, 0.f});
  Graphics gfx(wnd, cam, PIKSEL_SHADERS_PATH "/single_color.vert", PIKSEL_SHADERS_PATH "/single_color.frag");

  auto car = std::make_shared<Model>(PROJECT_PATH "/Bolid_F1.glb");
  auto ground = std::make_shared<Model>(PROJECT_PATH "/Bolid_Tor_test.glb");
  car->color = Color::Green;
  ground->color = Color::White;

  double scale_factor = 0.25;
  ground->scale = glm::scale(ground->scale, glm::vec3(scale_factor));

  gfx.addObject(car);
  gfx.addObject(ground);
  gfx.setBackground(Color::Blue);

  boink::CarModel car_model;
  car_model.front_left_wheel = Vector3d(-1.0, 0.0, 2.0);
  car_model.front_right_wheel = Vector3d(1.0, 0.0, 2.0);
  car_model.rear_left_wheel = Vector3d(-1.0, 0.0, -2.0);
  car_model.rear_right_wheel = Vector3d(1.0, 0.0, -2.0);
  car_model.max_steer_angle_deg = 30;

  World world(car_model);
  Entity::ID id = world.car_manager.addCar();

  world.start(0.0);

  CarInput input;
  input.brake = 0.0;
  input.steer_angle = 0.0;
  input.throttle = 0.0;
  world.car_manager.updateCar(id, input);
  const double simul_step = 0.02;

  float last = glfwGetTime();
  float simul_last = glfwGetTime();
  Window::MousePos prev_mouse_pos = wnd.getMousePos();

  bool is_gear_btn_pressed = false;
  while (wnd)
  {
    float right_now = glfwGetTime();
    float dt = right_now - last;
    last = right_now;

    if (wnd.getKey(GLFW_KEY_ESCAPE) == Window::KeyState::Press) wnd.close();

    if (wnd.getKey(GLFW_KEY_W) == Window::KeyState::Press)
    {
      cam.moveLongitudinal(cam_speed * dt);
    }
    if (wnd.getKey(GLFW_KEY_S) == Window::KeyState::Press)
    {
      cam.moveLongitudinal(-cam_speed * dt);
    }

    if (wnd.getKey(GLFW_KEY_A) == Window::KeyState::Press)
    {
      cam.moveLateral(-cam_speed * dt);
    }
    if (wnd.getKey(GLFW_KEY_D) == Window::KeyState::Press)
    {
      cam.moveLateral(cam_speed * dt);
    }

    auto [trans, input] = world.car_manager.getCarComponents<boink::Transform, CarInput>(id).value();

    if (wnd.getKey(GLFW_KEY_E) == Window::KeyState::Press)
    {
      input.steer_angle += 0.1f;
    }
    if (wnd.getKey(GLFW_KEY_Q) == Window::KeyState::Press)
    {
      input.steer_angle -= 0.1f;
    }

    if (wnd.getKey(GLFW_KEY_R) == Window::KeyState::Press)
    {
      input.throttle = 1;
      input.brake = 0;
    }
    else
    {
      input.throttle = 0;
    }
    if (wnd.getKey(GLFW_KEY_T) == Window::KeyState::Press)
    {
      input.brake = 1;
      input.throttle = 0;
    }
    else
    {
      input.brake = 0;
    }

    Window::MousePos mouse_pos = wnd.getMousePos();
    cam.rotateYaw((prev_mouse_pos.x - mouse_pos.x) * dt / 3.f);
    prev_mouse_pos.x = mouse_pos.x;
    cam.rotatePitch((prev_mouse_pos.y - mouse_pos.y) * dt / 3.f);
    prev_mouse_pos.y = mouse_pos.y;

    float simul_right_now = glfwGetTime();
    if (simul_right_now - simul_last > simul_step)
    {
      simul_last = simul_right_now;

      if (wnd.getKey(GLFW_KEY_Z) == Window::KeyState::Press)
      {
        if (!is_gear_btn_pressed)
        {
          input.gear_down = true;
          is_gear_btn_pressed = true;
        }
        else
        {
          input.gear_down = false;
        }
      }
      else
        input.gear_down = false;

      if (wnd.getKey(GLFW_KEY_X) == Window::KeyState::Press)
      {
        if (!is_gear_btn_pressed)
        {
          input.gear_up = true;
          is_gear_btn_pressed = true;
        }
        else
        {
          input.gear_up = false;
        }
      }
      else
        input.gear_up = false;

      world.car_manager.updateCar(id, input);
      world.update(simul_step);

      if (wnd.getKey(GLFW_KEY_X) == Window::KeyState::Release && wnd.getKey(GLFW_KEY_Z) == Window::KeyState::Release)
        is_gear_btn_pressed = false;
    }

    const auto& pos = trans.position;
    const auto& rot = trans.rotation;

    glm::mat4 rotate_mat(rot(0, 0), rot(1, 0), rot(2, 0), 0.f, rot(0, 1), rot(1, 1), rot(2, 1), 0.f, rot(0, 2),
                         rot(1, 2), rot(2, 2), 0.f, 0.f, 0.f, 0.f, 1.f);

    car->translate = glm::translate(glm::mat4(1.f), {pos.x(), pos.y(), pos.z()});
    car->rotate = glm::transpose(rotate_mat);

    ground->translate = glm::translate(glm::mat4(1.f), {0.f, -2.00f, 0.0f});
    ground->rotate = glm::mat4(1.f);
    ;

    gfx.render();
    wnd.update();
  }

  return 0;
}
