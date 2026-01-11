#include "boink/debug_render.h"

#include "boink/simulation.h"

#include "piksel/color.hh"
#include "piksel/model.hh"

#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>
#include <glm/matrix.hpp>
#include <memory>

#include <boink/utils/utility.h>
#include "boink/simulation/vehicle_model.h"

#include <glm/gtc/type_ptr.hpp>
#include <vector>

using namespace boink;
using namespace piksel;
using namespace Eigen;

std::vector<Mesh::Vertex> boink2piksel(const std::vector<btVector3> vertices);
int main()
{
  std::string_view car_model_path="Bolid_F1.glb";

  DebugRender dbg;
  //Simulation sim("Bolid_Tor_test.glb");
  //Simulation::ObjectID car_id=sim.addCar(car_model_path,800.);

  //sim.registerDebugDrawer(&dbg);
  
  ////////////// Car //////////////
  VehicleModel vehicle_model(car_model_path);
  Mesh chassis_mesh(
      "chassis",
      boink2piksel(vehicle_model.getChassis().vertices),
      vehicle_model.getChassis().indices);

  auto car = std::make_shared<Model>(1.f);
  car->addMesh(std::move(chassis_mesh));
  car->color=Color::White;
  car->translate=bt2glm(vehicle_model.getChassis().transform);
  car->scale=glm::mat4(1.f);
  car->rotate=glm::mat4(1.f);

  dbg.addObject(car);

  auto car2=std::make_shared<Model>(car_model_path,1.f);
  car2->color=Color::Green;
  dbg.addObject(car2);

  //Vehicle& vehicle=sim.getCar(car_id);

  //Track& track=sim.getTrack();
  //track.setPosition({5.f,0.f,0.f});

  dbg.getDeltaTime();
  while(dbg)
  {
    float dt=dbg.getDeltaTime();

    dbg.drawFrameOrigin();
    //sim.step(dt);
    dbg.update(dt);
  }
  return 0;
}

std::vector<Mesh::Vertex> boink2piksel(const std::vector<btVector3> vertices)
{
  std::vector<Mesh::Vertex> pik_vertices;

  for(const btVector3& vec:vertices)
  {
    Mesh::Vertex vertex;
    vertex.pos=bt2glm(vec);
    vertex.uv={0.f,0.f};
    
    pik_vertices.push_back(vertex);
  }
  return pik_vertices;
}

