#pragma once

#include "Eigen/Core"
#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/bolid_inputs.h"
#include "boink/components/bolid_model.h"
#include "boink/components/kinematics.h"

#include "boink/utils/math.h"

#include <algorithm>
#include <cmath>

namespace boink
{
  /**
   * @brief Simulates movement of a vehicle
   *
   */
  class MovementSystem
  {
  public:
    /**
      * @brief Updates all entities with Transform and Velocity components.
      *
      * @tparam Components_ The component types to that belong to ComponentManager.
      *
      * @param component_manager Reference to the ComponentManager storing components.
      * @param dt Delta time.
      */
    template<typename... Components_>
    void Update(ComponentManager<Components_...>& component_manager,
        double dt)
    {
      auto view=component_manager
        .template getComponentView<BolidModel,BolidInput,Transform,Kinematics>();
      view.forEach(
        [&, dt](
          const BolidModel& model,
          const BolidInput& input,
          Transform& trans, 
          Kinematics& kin
        )
        {
          using namespace Eigen;

          // Assumes that Y axis point oppostie to graviational force.
          // Assumes the right hand coordinates system.
          // Assumes that during delta time car moves with constant acceleration.
          // Assumes that postive angle means roaton counterclockwise.
          // Assumes that reference frames of a bolid is parallel to reference fram
          // of a world if not matrix of rotation must be provided
          // Assumes that initial bolid postion is at 0,0,0 if not displacement
          // vector must be provided

          // TODO Simulate engine output to the wheels
          // must be in other system
          double acc=input.throttle-input.brake;
          double wheel_turn=math::deg2rad(input.steer_angle*30);

          Vector3d wheel_direction=
            math::getYRotationMatrix(wheel_turn)*model.direction;
          wheel_direction.normalize();

          // Simulate car movement
          kin.velocity=(kin.velocity.norm()+acc*dt)*wheel_direction;

          Vector3d front_displacement=
            kin.velocity*dt+
            (0.5*acc*dt*dt)*wheel_direction;

          // Calculate bolid displacement
          // TODO
          // For now we know that the angle will be in XZ plain
          // but if car will start go pitch or roll then this angle will not
          // be in any of coordinate planes this will have to be done be composition
          // of rotations.
          //Vector3d temp_a=front_car-rear_car;
          Vector3d temp_b=model.direction+front_displacement;

          // TODO 
          // Because of how doubles operate |cos_theta_y| might be greater than
          // one this will cause std::acos to return NaN to avoid for now
          // we use std::clamp to make sure that the value is in correct range
          // but this is slow so improvements must be made in the near future
          // Also std::acos returns value between [0;PI] so we must have some way
          // to tell in which direction should be bolid rotated counter clockwise
          // or clockwise.
          double cos_theta_y=
            model.direction.dot(temp_b)/
            (model.direction.norm()*temp_b.norm());

          cos_theta_y=std::clamp(cos_theta_y,-1.0,1.0);
          double theta_y=std::acos(cos_theta_y);

          //Normally we will be using cross product but in 2D we can 
          //optimize like this 
          double rotation_direction=
            model.direction.x()*temp_b.z()-
            temp_b.x()*model.direction.z();
          rotation_direction/=std::abs(rotation_direction);
          // The only problem with solution is that if vectors are perpendicular
          // we got 0 or sth close to zero and one time it will mean rotate clockwise
          // anthoer the other way
          
          theta_y*=rotation_direction;

          // to jest na chuja narazie
          Vector3d bolid_displacement=
            model.front+math::getYRotationMatrix(theta_y)*front_displacement;
          trans.position+=bolid_displacement;
        }
      );
    }
  };
}
