#include "boink/debug_render.h"

#include "boink/simulation.h"

#include "piksel/color.hh"
#include "piksel/vehicle_model.hh"

#include <LinearMath/btTransform.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Core>
#include <glm/matrix.hpp>
#include <memory>

#include <boink/utils/utility.h>
#include "boink/simulation/vehicle_mesh.h"

#include <glm/gtc/type_ptr.hpp>
#include <vector>

using namespace boink;
using namespace piksel;

std::vector<Mesh::Vertex> boink2piksel(const std::vector<btVector3> vertices);
std::shared_ptr<VehicleModel> createVehicleModel(const VehicleMesh& vehicle_mesh);
int main()
{
  std::string_view car_model_path="Bolid_F1.glb";

  DebugRender dbg;
  Simulation sim("Bolid_Tor_test.glb");

  sim.registerDebugDrawer(&dbg);
  
  ////////////// Car //////////////
  VehicleMesh vehicle_mesh(car_model_path);
  Simulation::ObjectID car_id=sim.addCar({
      .mesh=std::shared_ptr<VehicleMesh>(&vehicle_mesh,[](const VehicleMesh*){}),
      .mass=800.,
      .wheel_radius=0.36f,
      .suspension_rest_length=0.42f,
      .center_of_mass={0.f,1.f,0.f}});

  auto vehicle_model=createVehicleModel(vehicle_mesh);
  dbg.addObject(vehicle_model);

  Vehicle& vehicle=sim.getCar(car_id);
  vehicle.setPosition({0.f,23.f,0.f});

  Track& track=sim.getTrack();
  track.setPosition({5.f,0.f,0.f});

  dbg.getDeltaTime();
  while(dbg)
  {
    float dt=dbg.getDeltaTime();

    vehicle_model->setTransform(bt2glm(vehicle.getChassisWorldTransform()));
    vehicle_model->setWheelWorldTransform(
        VehicleModel::WheelPosition::RearLeft,
        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::RearLeft)));
    vehicle_model->setWheelWorldTransform(
        VehicleModel::WheelPosition::RearRight,
        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::RearRight)));
    vehicle_model->setWheelWorldTransform(
        VehicleModel::WheelPosition::FrontLeft,
        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::FrontLeft)));
    vehicle_model->setWheelWorldTransform(
        VehicleModel::WheelPosition::FrontRight,
        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::FrontRight)));

    dbg.drawFrameOrigin();
    sim.step(dt);
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

std::shared_ptr<VehicleModel> createVehicleModel(const VehicleMesh& vehicle_mesh)
{
  Mesh chassis_mesh(
      "",
      boink2piksel(vehicle_mesh.getChassis().vertices),
      vehicle_mesh.getChassis().indices);
  Mesh wheel_rear_left_mesh(
      "",
      boink2piksel(vehicle_mesh.getWheel(WheelPosition::RearLeft).vertices),
      vehicle_mesh.getWheel(WheelPosition::RearLeft).indices);
  Mesh wheel_rear_right_mesh(
      "",
      boink2piksel(vehicle_mesh.getWheel(WheelPosition::RearRight).vertices),
      vehicle_mesh.getWheel(WheelPosition::RearRight).indices);
  Mesh wheel_front_left_mesh(
      "",
      boink2piksel(vehicle_mesh.getWheel(WheelPosition::FrontLeft).vertices),
      vehicle_mesh.getWheel(WheelPosition::FrontLeft).indices);
  Mesh wheel_front_right_mesh(
      "",
      boink2piksel(vehicle_mesh.getWheel(WheelPosition::FrontRight).vertices),
      vehicle_mesh.getWheel(WheelPosition::FrontRight).indices);

  auto vehicle=std::make_shared<VehicleModel>(
    std::move(wheel_rear_left_mesh),
    std::move(wheel_rear_right_mesh),
    std::move(wheel_front_left_mesh),
    std::move(wheel_front_right_mesh),
    std::move(chassis_mesh));
  vehicle->color=Color{{1.f,0.f,1.f,1.f}};

  return vehicle;
}
