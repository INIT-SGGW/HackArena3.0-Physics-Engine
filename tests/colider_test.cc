#include "boink/debug_render.h"

#include "boink/simulation.h"

#include "piksel/color.hh"
#include "piksel/model.hh"

#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>
#include <glm/matrix.hpp>
#include <memory>

#include <boink/utils/utility.h>

#include <glm/gtc/type_ptr.hpp>

using namespace boink;
using namespace piksel;
using namespace Eigen;

int main()
{
  std::string_view car_model_path="Bolid_F1.glb";
  DebugRender dbg;

  Simulation sim("Bolid_Tor_test.glb");
  //sim.addSphere(5.f,{50.f,150.f,0.f});

  Simulation::ObjectID car_id=sim.addCar(car_model_path,800.);

  sim.registerDebugDrawer(&dbg);
  
  ////////////// Car //////////////
  auto car = std::make_shared<Model>(car_model_path,1.f);
  car->color=Color::White;
  //const Mesh& mesh=car->getMeshes()[0];
  //std::shared_ptr<Mesh> chuj(const_cast<Mesh*>(&mesh),[](const Mesh*){});
  //dbg.addObject(chuj);
  dbg.addObject(car);

  //glm::mat4 mesh_transform=mesh.getTransform();
  //glm::mat4 transform=car->getTransform()*mesh_transform;
  //glUniformMatrix4fv(
  //    glGetUniformLocation(dbg.gfx_.shader_.get(),"trans"),
  //    1,GL_FALSE,glm::value_ptr(transform));
  //glUniform3f(
  //    glGetUniformLocation(dbg.gfx_.shader_.get(),"color"),
  //    1.f,1.f,0.f);

  //glLineWidth(1.f);

  btVector3 pos=btVector3(0.f,0.f,0.f);
  //btVector3 edge(0.f,car_model.radius,0.f);
  btVector3 edge(0.f,0.f,0.f);
  
  Vehicle& vehicle=sim.getCar(car_id);
  auto it=std::find_if(car->getMeshes().cbegin(),car->getMeshes().cend(),
      [](const Mesh& mesh)
      {
        return mesh.getName()==Vehicle::BODY_NAME;
      }
  );
  auto body_translate=it->translate;

  Track& track=sim.getTrack();
  track.setPosition({5.f,0.f,0.f});
  dbg.getDeltaTime();
  while(dbg)
  {
    float dt=dbg.getDeltaTime();
    auto [translate,rotation,scale]=decomposeMatrix(
        bt2glm(vehicle.getWorldTransform()));

    car->translate=translate;
    car->rotate=rotation;
    car->scale=scale;

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
