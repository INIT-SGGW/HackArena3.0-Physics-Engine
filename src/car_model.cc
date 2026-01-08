#include "boink/components/car_model.h"

namespace boink
{
  CarModel::CarModel(
      double mass,
      double radius,
      double steer_angle_deg,
      std::string_view filepath)
    :
      model_(filepath,1.f),
      mass_(mass),
      max_steer_angle_deg_(steer_angle_deg),
      radius_(radius)
  {
    for(const auto& mesh : model_.getMeshes())
    {
      glm::vec3 avg_vec(0.f);
      auto vertices=mesh.getVertices();
      for(const auto& vertex:vertices)
        avg_vec+=vertex.pos;

      avg_vec/=vertices.size();

      glm::vec3 vec=avg_vec+glm::vec3(mesh.translate[3]);
      if(mesh.getName()==CarModel::REAR_LEFT_WHEEL_NAME)
        rear_left_wheel_=btVector3(vec.x,vec.y,vec.z);
      else if(mesh.getName()==CarModel::REAR_RIGHT_WHEEL_NAME)
        rear_right_wheel_=btVector3(vec.x,vec.y,vec.z);
      else if(mesh.getName()==CarModel::FRONT_LEFT_WHEEL_NAME)
        front_left_wheel_=btVector3(vec.x,vec.y,vec.z);
      else if(mesh.getName()==CarModel::FRONT_RIGHT_WHEEL_NAME)
        front_right_wheel_=btVector3(vec.x,vec.y,vec.z);
    }
    front_=
      0.5*(front_left_wheel_-front_right_wheel_)+
      front_right_wheel_;

    btVector3 temp_rear=
      0.5*(rear_left_wheel_-rear_right_wheel_)+
      rear_right_wheel_;
    direction_=front_-temp_rear;
    direction_.normalize();

    btVector3 temp=rear_left_wheel_ -rear_right_wheel_;
    
    // It should always point to the top of a car
    normal_=direction_.cross(temp);
    normal_.normalize();
  }
}
