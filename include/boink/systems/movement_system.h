#pragma once

#include "Eigen/Core"
#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/bolid_inputs.h"
#include "boink/components/bolid_model.h"
#include "boink/components/kinematics.h"

#include "boink/utils/math.h"

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

          // Assumes that Y axis points to graviational force.
          // Assumes the right hand coordinates system.
          // Assumes that during delta time car moves with constant acceleration.
          // Assumes that postive angle means roaton counterclockwise.
          // this means that angle_x is postive when rotation is from Y to Z, for
          // angle_y is from Z to X and for angle_z from X to Y.
          
          // Assumes that initial bolid postion is at 0,0,0 if not displacement
          // vector must be provided

          // TODO Simulate engine output to the wheels
          // must be in other system
          kin.acceleration=input.throttle-input.brake;
          double turn_angle=
            math::deg2rad(input.steer_angle/2 * model.max_steer_angle_deg);

          double sin_turn=std::sin(turn_angle);
          double cos_turn=std::cos(turn_angle);
          Vector3d wheel_direction=
            math::getRodriguesRotationMatrix(sin_turn,cos_turn,model.normal)*
            model.direction;

          // Simulate car movement
          kin.velocity=(kin.velocity.norm()+kin.acceleration*dt)*wheel_direction;

          Vector3d front_displacement=
            kin.velocity*dt+
            (0.5*kin.acceleration*dt*dt)*wheel_direction;

          Matrix3d delta_rotation=
            math::getRotationMatrix(
                model.direction,model.direction+front_displacement);

          Vector3d new_front=model.front+front_displacement;
          Vector3d front_new_disp_after_d_rot=new_front-delta_rotation*new_front;

          // Its rotation matrix so traspose equals to inverse.
          trans.position=trans.rotation.transpose()*front_new_disp_after_d_rot;

          // Order matters
          trans.rotation=delta_rotation*trans.rotation;
        }
      );
    }
  };
}
