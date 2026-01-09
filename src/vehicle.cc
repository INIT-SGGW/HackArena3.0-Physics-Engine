#include "boink/simulation/vehicle.h"
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <piksel/model.hh>
#include "boink/utils/utility.h"

namespace boink
{
  Vehicle::Vehicle(
      std::string_view filename, 
      btScalar mass,
      std::shared_ptr<btDynamicsWorld> world)
    :world_(world),raycaster_(new btDefaultVehicleRaycaster(world_.get()))
  {
    piksel::Model model(filename,1.f);

    // TODO do this once not for every vehicle created
    std::vector<piksel::Mesh::Vertex> mesh_vertices;
    glm::mat4 mesh_transform;
    btVector3 rear_left_wheel{};
    btVector3 rear_right_wheel{};
    btVector3 front_left_wheel{};
    btVector3 front_right_wheel{};
    for(const auto& mesh : model.getMeshes())
    {
      if(mesh.getName()==BODY_NAME)
      {
        mesh_vertices=mesh.getVertices();
        mesh_transform=mesh.getTransform();
      }

      glm::vec3 avg_vec(0.f);
      auto vertices=mesh.getVertices();
      for(const auto& vertex:vertices)
        avg_vec+=vertex.pos;

      avg_vec/=vertices.size();

      glm::vec3 vec=avg_vec+glm::vec3(mesh.translate[3]);
      if(mesh.getName()==REAR_LEFT_WHEEL_NAME)
        rear_left_wheel=btVector3(vec.x,vec.y,vec.z);
      else if(mesh.getName()==REAR_RIGHT_WHEEL_NAME)
        rear_right_wheel=btVector3(vec.x,vec.y,vec.z);
      else if(mesh.getName()==FRONT_LEFT_WHEEL_NAME)
        front_left_wheel=btVector3(vec.x,vec.y,vec.z);
      else if(mesh.getName()==FRONT_RIGHT_WHEEL_NAME)
        front_right_wheel=btVector3(vec.x,vec.y,vec.z);
    }

    std::vector<btVector3> vertices; 
    vertices.reserve(mesh_vertices.size());
    std::transform(
        mesh_vertices.cbegin(),mesh_vertices.cend(),
        std::back_inserter(vertices),
        [](const piksel::Mesh::Vertex& vertex)
        {
          return glm2bt(vertex.pos);
        }
    );
    auto [scale,transform]=glm2bt(mesh_transform);

    collision_shape_=createCollisonShape(scale,vertices);
    rigidbody_=createRigidbody(mass);

    vehicle_=std::unique_ptr<btRaycastVehicle>(
        new btRaycastVehicle(tuning_, rigidbody_.get(), raycaster_.get())
    );

    vehicle_->setCoordinateSystem(
        0, // right (X)
        1, // up (Y)
        2  // forward (Z)
    );

    world_->addVehicle(vehicle_.get());

    btVector3 wheelDirectionCS0(0, -1, 0);
    btVector3 wheelAxleCS(-1, 0, 0);

    btScalar suspensionRestLength = 0.2f;
    btScalar wheelRadius = 0.36f;
    btVector3 wheel_center_ajust=
      wheelDirectionCS0*(suspensionRestLength+wheelRadius);

    bool isFrontWheel = true;
    
    // Front-left
    vehicle_->addWheel(
        front_left_wheel-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning_,
        isFrontWheel
    );

    // Front-right
    vehicle_->addWheel(
        front_right_wheel-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning_,
        isFrontWheel
    );

    isFrontWheel = false;

    // Rear-left
    vehicle_->addWheel(
        rear_left_wheel-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning_,
        isFrontWheel
    );

    // Rear-right
    vehicle_->addWheel(
        rear_right_wheel-wheel_center_ajust,
        wheelDirectionCS0,
        wheelAxleCS,
        suspensionRestLength,
        wheelRadius,
        tuning_,
        isFrontWheel
    );
  }

  Vehicle::~Vehicle() noexcept
  {
    world_->removeVehicle(vehicle_.get());
    world_->removeRigidBody(rigidbody_.get());
  }

  std::unique_ptr<btCollisionShape> Vehicle::createCollisonShape(
      const btVector3& scale,
      const std::vector<btVector3>& vertices)
  {
    std::unique_ptr<btConvexHullShape> hull(new btConvexHullShape());

    for (const btVector3& v : vertices)
    {
        hull->addPoint(v, false);
    }

    hull->recalcLocalAabb();
    hull->optimizeConvexHull();
    hull->initializePolyhedralFeatures();

    hull->setLocalScaling(scale);

    return hull;
  }

  std::unique_ptr<btRigidBody> Vehicle::createRigidbody(
      btScalar mass)
  {
    assert(mass!=0.f);

    btVector3 local_inertia(0, 0, 0);
    collision_shape_->calculateLocalInertia(mass, local_inertia);

    btRigidBody::btRigidBodyConstructionInfo rb_info
      (mass, motion_state_.get(), collision_shape_.get(), local_inertia);

    std::unique_ptr<btRigidBody> body (new btRigidBody(rb_info));

    world_->addRigidBody(body.get());

    return body;
  }
}
