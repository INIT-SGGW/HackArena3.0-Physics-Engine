#include <GLFW/glfw3.h>
#include <LinearMath/btTransform.h>
#include <boink/utils/utility.h>

#include <algorithm>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/matrix.hpp>
#include <memory>
#include <vector>

#include "boink/debug_drawer.h"
#include "boink/gltf_extractor.h"
#include "boink/simulation.h"
#include "boink/simulation/vehicle_mesh.h"
#include "piksel/vehicle_model.hh"

using namespace boink;
using namespace piksel;

void drawCenterline(DebugDrawer& dbg, const GltfExtractor::Node& node, float elapsed_time);
void handleVehicle(Vehicle& vehicle, const DebugDrawer& dbg);
void updateTransform(std::shared_ptr<VehicleModel> vehicle_model, const Vehicle& vehicle);
std::vector<Mesh::Vertex> boink2piksel(const std::vector<btVector3> vertices);
VehicleModel createVehicleModel(std::shared_ptr<VehicleMesh> vehicle_mesh);
int main()
{
  std::string_view car_model_path = "Bolid_F1.glb";

  DebugDrawer dbg;
  std::string_view track_filename = "lowpoly_track_1_test_5.glb";
  Simulation sim(track_filename);

  sim.registerDebugDrawer(&dbg);

  ////////////// Vehicle //////////////
  std::shared_ptr<VehicleMesh> vehicle_mesh0 = std::make_shared<VehicleMesh>(car_model_path);

  std::shared_ptr<VehicleMesh> vehicle_mesh1 = std::make_shared<VehicleMesh>(car_model_path);

  btRaycastVehicle::btVehicleTuning tuning;
  tuning.m_frictionSlip = 3.5f;
  tuning.m_maxSuspensionForce = 20000.;
  tuning.m_maxSuspensionTravelCm = 8.;
  tuning.m_suspensionCompression = 10.;
  tuning.m_suspensionDamping = 10.;
  tuning.m_suspensionStiffness = 50.;

  Simulation::ObjectID car_id0 = sim.addVehicle({.mesh = vehicle_mesh0,
                                                 .mass = 800.,
                                                 .wheel_radius = 0.33,
                                                 .suspension_rest_length = 1.02,
                                                 .max_steer_angle = 1.5,
                                                 .center_of_mass = {0., -0.4, 0.},
                                                 .tuning = tuning});
  sim.getVehicle(car_id0).setPosition({0., 13., 7.});

  std::shared_ptr<VehicleModel> vehicle_model0(new VehicleModel(createVehicleModel(vehicle_mesh0)));
  dbg.addObject(vehicle_model0);

  Simulation::ObjectID car_id1 = sim.addVehicle({.mesh = vehicle_mesh1,
                                                 .mass = 800.,
                                                 .wheel_radius = 0.33,
                                                 .suspension_rest_length = 0.32,
                                                 .max_steer_angle = 1.5,
                                                 .center_of_mass = {0., -0.4, 0.},
                                                 .tuning = tuning});
  sim.getVehicle(car_id1).setPosition({0., 23., 0.});

  std::shared_ptr<VehicleModel> vehicle_model1(new VehicleModel(*vehicle_model0));
  dbg.addObject(vehicle_model1);

  ///////////////// Track /////////////////////

  Track& track = sim.getTrack();
  // track.setPosition({5.,0.,0.});
  auto centerline_points_dist2 = track.getCenterline().getPointsAndDist();
  std::vector<btVector3> centerline_points(centerline_points_dist2.size());
  std::generate(centerline_points.begin(), centerline_points.end(),
                [&, i = 0]() mutable { return centerline_points_dist2[i++].first; });

  GltfExtractor extractor(track_filename);
  const auto& node = extractor.getNode("Centerline");

  float prev = dbg.getTime();
  float elapsed_time = 0.0;
  while (dbg)
  {
    float now = dbg.getTime();
    float dt = now - prev;
    prev = now;
    elapsed_time += dt;

    handleVehicle(sim.getVehicle(car_id1), dbg);

    sim.step(dt);
    dbg.drawFrameOrigin();
    // drawCenterline(dbg,node,elapsed_time);
    dbg.drawLines(centerline_points, {0.2, 0.5, 0.5}, elapsed_time * 1000);

    updateTransform(vehicle_model0, sim.getVehicle(car_id0));
    updateTransform(vehicle_model1, sim.getVehicle(car_id1));

    const auto& trans = sim.getVehicle(car_id1).getWorldTransform();
    btVector3 back = trans.getBasis() * btVector3(0, 0.25, 1);
    btVector3 cam_pos = trans.getOrigin() + back * 15;

    dbg.setCamera(cam_pos, trans.getOrigin());
    dbg.update();
  }
  return 0;
}

void updateTransform(std::shared_ptr<VehicleModel> vehicle_model, const Vehicle& vehicle)
{
  vehicle_model->setChassisWorldTransform(bt2glm(vehicle.getChassisWorldTransform()));
  vehicle_model->setWheelWorldTransform(VehicleModel::WheelPosition::RearLeft,
                                        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::RearLeft)));
  vehicle_model->setWheelWorldTransform(VehicleModel::WheelPosition::RearRight,
                                        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::RearRight)));
  vehicle_model->setWheelWorldTransform(VehicleModel::WheelPosition::FrontLeft,
                                        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::FrontLeft)));
  vehicle_model->setWheelWorldTransform(VehicleModel::WheelPosition::FrontRight,
                                        bt2glm(vehicle.getWheelWorldTransform(WheelPosition::FrontRight)));
}

std::vector<Mesh::Vertex> boink2piksel(const std::vector<btVector3> vertices)
{
  std::vector<Mesh::Vertex> pik_vertices;

  for (const btVector3& vec : vertices)
  {
    Mesh::Vertex vertex;
    vertex.pos = bt2glm(vec);

    pik_vertices.push_back(vertex);
  }
  return pik_vertices;
}

VehicleModel createVehicleModel(std::shared_ptr<VehicleMesh> vehicle_mesh)
{
  std::shared_ptr<Mesh> chassis_mesh =
      std::make_shared<Mesh>(boink2piksel(vehicle_mesh->getChassis().vertices), vehicle_mesh->getChassis().indices);
  std::shared_ptr<Mesh> wheel_rear_left_mesh =
      std::make_shared<Mesh>(boink2piksel(vehicle_mesh->getWheel(WheelPosition::RearLeft).vertices),
                             vehicle_mesh->getWheel(WheelPosition::RearLeft).indices);
  std::shared_ptr<Mesh> wheel_rear_right_mesh =
      std::make_shared<Mesh>(boink2piksel(vehicle_mesh->getWheel(WheelPosition::RearRight).vertices),
                             vehicle_mesh->getWheel(WheelPosition::RearRight).indices);
  std::shared_ptr<Mesh> wheel_front_left_mesh =
      std::make_shared<Mesh>(boink2piksel(vehicle_mesh->getWheel(WheelPosition::FrontLeft).vertices),
                             vehicle_mesh->getWheel(WheelPosition::FrontLeft).indices);
  std::shared_ptr<Mesh> wheel_front_right_mesh =
      std::make_shared<Mesh>(boink2piksel(vehicle_mesh->getWheel(WheelPosition::FrontRight).vertices),
                             vehicle_mesh->getWheel(WheelPosition::FrontRight).indices);

  VehicleModel vehicle(wheel_rear_left_mesh, wheel_rear_right_mesh, wheel_front_left_mesh, wheel_front_right_mesh,
                       chassis_mesh);

  return vehicle;
}

void handleVehicle(Vehicle& vehicle, const DebugDrawer& dbg) {}
void drawCenterline(DebugDrawer& dbg, const GltfExtractor::Node& node, float elapsed_time)
{
  const auto& vertices = node.vertices;
  const auto& indices = node.indices;

  for (int i = 0; i < indices.size(); i += 2)
  // for(int i=0;i<elapsed_time/5;i+=2)
  {
    dbg.drawLine(vertices[indices[i]], vertices[indices[i + 1]], {0.5, 0.2, 0.5});
  }
}
