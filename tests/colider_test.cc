#include "boink/debug_drawer.h"
#include "boink/simulation.h"

#include "piksel/vehicle_model.hh"

#include <GLFW/glfw3.h>
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

void handleCar(Vehicle& vehicle,const DebugDrawer& dbg);
void updateTransform(
    std::shared_ptr<VehicleModel> vehicle_model, 
    const Vehicle& vehicle);
std::vector<Mesh::Vertex> boink2piksel(const std::vector<btVector3> vertices);
VehicleModel createVehicleModel(
    std::shared_ptr<VehicleMesh>vehicle_mesh);
int main()
{
  std::string_view car_model_path="Bolid_F1.glb";

  DebugDrawer dbg;
  Simulation sim("Bolid_Tor_test.glb");

  sim.registerDebugDrawer(&dbg);
  
  ////////////// Car //////////////
  std::shared_ptr<VehicleMesh> vehicle_mesh0=
    std::make_shared<VehicleMesh>(car_model_path);

  std::shared_ptr<VehicleMesh> vehicle_mesh1=
    std::make_shared<VehicleMesh>(car_model_path);

  Simulation::ObjectID car_id0=sim.addCar({
      .mesh=vehicle_mesh0,
      .mass=800.,
      .wheel_radius=0.36f,
      .suspension_rest_length=1.02f,
      .center_of_mass={0.f,1.f,0.f}});
  sim.getCar(car_id0).setPosition({0.f,13.f,7.f});

  std::shared_ptr<VehicleModel> vehicle_model0(
      new VehicleModel(createVehicleModel(vehicle_mesh0)));
  dbg.addObject(vehicle_model0);

  Simulation::ObjectID car_id1=sim.addCar({
      .mesh=vehicle_mesh1,
      .mass=800.,
      .wheel_radius=0.36f,
      .suspension_rest_length=0.52f,
      .center_of_mass={0.f,1.f,0.f}});
  sim.getCar(car_id1).setPosition({0.f,23.f,0.f});

  std::shared_ptr<VehicleModel> vehicle_model1(
      new VehicleModel(*vehicle_model0));
  dbg.addObject(vehicle_model1);

  ///////////////// Track /////////////////////

  Track& track=sim.getTrack();
  track.setPosition({5.f,0.f,0.f});

  dbg.getDeltaTime();
  while(dbg)
  {
    float dt=dbg.getDeltaTime();

    handleCar(sim.getCar(car_id1),dbg);

    updateTransform(vehicle_model0,sim.getCar(car_id0));
    updateTransform(vehicle_model1,sim.getCar(car_id1));

    dbg.drawFrameOrigin();
    sim.step(dt);
    dbg.update(dt);
  }
  return 0;
}

void updateTransform(
    std::shared_ptr<VehicleModel> vehicle_model, 
    const Vehicle& vehicle)
{
    vehicle_model->setChassisWorldTransform(
        bt2glm(vehicle.getChassisWorldTransform()));
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
}

std::vector<Mesh::Vertex> boink2piksel(const std::vector<btVector3> vertices)
{
  std::vector<Mesh::Vertex> pik_vertices;

  for(const btVector3& vec:vertices)
  {
    Mesh::Vertex vertex;
    vertex.pos=bt2glm(vec);
    
    pik_vertices.push_back(vertex);
  }
  return pik_vertices;
}

VehicleModel createVehicleModel(
    std::shared_ptr<VehicleMesh>vehicle_mesh)
{
  std::shared_ptr<Mesh> chassis_mesh=std::make_shared<Mesh>(
      boink2piksel(vehicle_mesh->getChassis().vertices),
      vehicle_mesh->getChassis().indices);
  std::shared_ptr<Mesh> wheel_rear_left_mesh=std::make_shared<Mesh>(
      boink2piksel(vehicle_mesh->getWheel(WheelPosition::RearLeft).vertices),
      vehicle_mesh->getWheel(WheelPosition::RearLeft).indices);
  std::shared_ptr<Mesh> wheel_rear_right_mesh=std::make_shared<Mesh>(
      boink2piksel(vehicle_mesh->getWheel(WheelPosition::RearRight).vertices),
      vehicle_mesh->getWheel(WheelPosition::RearRight).indices);
  std::shared_ptr<Mesh> wheel_front_left_mesh=std::make_shared<Mesh>(
      boink2piksel(vehicle_mesh->getWheel(WheelPosition::FrontLeft).vertices),
      vehicle_mesh->getWheel(WheelPosition::FrontLeft).indices);
  std::shared_ptr<Mesh> wheel_front_right_mesh=std::make_shared<Mesh>(
      boink2piksel(vehicle_mesh->getWheel(WheelPosition::FrontRight).vertices),
      vehicle_mesh->getWheel(WheelPosition::FrontRight).indices);

  VehicleModel vehicle(
    wheel_rear_left_mesh,
    wheel_rear_right_mesh,
    wheel_front_left_mesh,
    wheel_front_right_mesh,
    chassis_mesh);

  return vehicle;
}

void handleCar(Vehicle& vehicle,const DebugDrawer& dbg)
{
  if(dbg.getKey(GLFW_KEY_UP)==Window::KeyState::Press)
    vehicle.setEngineForce(500.f);
  else
    vehicle.setEngineForce(0.f);

  if(dbg.getKey(GLFW_KEY_DOWN)==Window::KeyState::Press)
    vehicle.setBrake(30);
  else
    vehicle.setBrake(0.f);

  if(dbg.getKey(GLFW_KEY_LEFT)==Window::KeyState::Press)
    vehicle.setSteering(0.3f,Vehicle::TurnDirection::Left);
  else if(dbg.getKey(GLFW_KEY_RIGHT)==Window::KeyState::Press)
    vehicle.setSteering(0.3f,Vehicle::TurnDirection::Right);
  else
    vehicle.setSteering(0.0f,Vehicle::TurnDirection::Right);

}
