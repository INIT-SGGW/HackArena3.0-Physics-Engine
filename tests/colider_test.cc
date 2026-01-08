#include "boink/debug_render.h"

#include "boink/simulation.h"

#include "piksel/color.hh"
#include "piksel/model.hh"

#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>
#include <memory>

#include <glm/gtc/type_ptr.hpp>

using namespace boink;
using namespace piksel;
using namespace Eigen;

int main()
{
  std::string_view car_model_path="Bolid_F1.glb";
  DebugRender dbg;

  boink::CarModel car_model(800.,0.36,30.,car_model_path);

  Simulation sim;
  btVector3 dims(100.f,100.f,100.f);
  sim.addGround(dims,{0.f,0.f,0.f});
  //sim.addSphere(5.f,{50.f,150.f,0.f});

  sim.addCar(car_model);

  sim.registerDebugDrawer(&dbg);
  
  ////////////// Car //////////////
  auto car = std::make_shared<Model>(car_model_path,1.f);
  car->color=Color::White;
  //const Mesh& mesh=car->getMeshes()[0];
  //std::shared_ptr<Mesh> chuj(const_cast<Mesh*>(&mesh),[](const Mesh*){});
  //dbg.addObject(chuj);
  //dbg.addObject(car);

  //glm::mat4 mesh_transform=mesh.getTransform();
  //glm::mat4 transform=car->getTransform()*mesh_transform;
  //glUniformMatrix4fv(
  //    glGetUniformLocation(dbg.gfx_.shader_.get(),"trans"),
  //    1,GL_FALSE,glm::value_ptr(transform));
  //glUniform3f(
  //    glGetUniformLocation(dbg.gfx_.shader_.get(),"color"),
  //    1.f,1.f,0.f);

  //glLineWidth(1.f);

  btVector3 pos={
    (float)car_model.getRearRightWheel().x(),
    (float)car_model.getRearRightWheel().y(),
    (float)car_model.getRearRightWheel().z()};

  //btVector3 edge(0.f,car_model.radius,0.f);
  btVector3 edge(0.f,0.f,0.f);
  
  dbg.getDeltaTime();
  while(dbg)
  {
    float dt=dbg.getDeltaTime();

    //// Render axis
    dbg.drawLine(
        pos+edge,
        {10.f,0.f,0.f},
        {1.f,0.f,0.f});
    dbg.drawLine(
        pos+edge,
        {0.f,10.f,0.f},
        {0.f,1.f,0.f});
    dbg.drawLine(
        pos+edge,
        {0.f,0.f,10.f},
        {0.f,0.f,1.f});
    sim.step(dt);
    dbg.update(dt);
  }
  return 0;
}
