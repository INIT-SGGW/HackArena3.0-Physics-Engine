#include "boink/components/car_inputs.h"
#include "boink/components/rigidbody.h"
#include "boink/components/transform.h"
#include "boink/debug_render.h"

#include "boink/world.h"

#include "piksel/color.hh"
#include "piksel/model.hh"

#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>

using namespace boink;
using namespace piksel;
using namespace Eigen;
int main()
{
  std::string_view car_model_path="Bolid_F1.glb";
  DebugRender dbg;

  boink::CarModel car_model(0,car_model_path);
  car_model.front_left_wheel=Vector3d(-1.0,0.0,2.0);
  car_model.front_right_wheel=Vector3d(1.0,0.0,2.0);
  car_model.rear_left_wheel=Vector3d(-1.0,0.0,-2.0);
  car_model.rear_right_wheel=Vector3d(1.0,0.0,-2.0);
  car_model.max_steer_angle_deg=30;

  World world(std::move(car_model));
  btVector3 dims(100.f,100.f,100.f);
  world.addGround(dims,{0.f,0.f,0.f});
  world.addSphere(5.f,{50.f,150.f,0.f});

  world.addCar(CarInput{},Rigidbody{},boink::Transform{},Kinematics{});

  world.setDebuger(&dbg);
  world.start(0.0);
  
  ////////////// Car //////////////
  auto car = std::make_shared<Model>(car_model_path,1.f);
  car->color=Color::White;
  //float scale_factor=12.25;
  //car->scale=glm::scale(car->scale,glm::vec3(1.f)*scale_factor);
  dbg.addObject(car);
  glm::mat4 chuj=car->getTransform();
  
  dbg.getDeltaTime();
  while(dbg)
  {
    float dt=dbg.getDeltaTime();
    world.update(dt);
    dbg.update(dt);
  }
  return 0;
}
