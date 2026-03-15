#pragma once

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btVector3.h>

#include <glm/detail/qualifier.hpp>
#include <glm/mat4x4.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace boink::math
{
  glm::mat4 bt2glm(const btTransform& bt_trans);
  glm::mat3 bt2glm(const btMatrix3x3& bt_mat);
  glm::vec3 bt2glm(const btVector3& bt_vec);

  std::pair<btVector3,btTransform> glm2bt(const glm::mat4& glm_mat);
  btMatrix3x3 glm2bt(const glm::mat3& glm_mat);
  btVector3 glm2bt(const glm::vec3& glm_vec);

  std::tuple<glm::mat4,glm::mat4,glm::mat4> 
  decomposeMatrix(const glm::mat4& transform);

  bool areColinear(
      const btVector3& a, const btVector3& b, btScalar epsilon=1e-6);
  size_t getIthClosestIndex(const std::vector<btVector3>& vec,
      const btVector3& point,size_t ith);

  btVector3 vec3toLocalvec2(
      const btVector3& vec,
      const btVector3& local_x,
      const btVector3& local_y,
      const btVector3& local_origin);

  /**
   * @brief Calculates point of intersection in a given plane.
   *
   * @param ray_dir raycast direction
   * @param ray_start starting point of raycast
   * @param normal Plane normal vector.
   * @param a first point of the line.
   * @param b second point of the line.
   * @param epsilon Accuracy of calculation.
   *
   * @return Point of intersection, distance from ray_start to
   * point of intersection and distance from point a to a point of intersection 
   * in unit of (b-a). 
   * If second parameter is not greater than or equal 0 means the line intersection
   * is in opposite direction.
   * If last return parameter is not between [0;1] it means that
   * point of intersection is not between points a and b.
   * If returned std::nullopt means the ray and line are parallel
   */
  std::optional<std::tuple<btVector3,btScalar,btScalar>> getRayLineInterscetion(
      btVector3 ray_dir,
      btVector3 ray_start,
      btVector3 normal,
      btVector3 a,
      btVector3 b,
      btScalar epsilon=1e-4);

}

template<>
struct std::hash<btVector3>
{
  std::size_t operator()(const btVector3& v) const noexcept
  {
    size_t h_x=std::hash<btScalar>{}(v.x());
    size_t h_y=std::hash<btScalar>{}(v.y());
    size_t h_z=std::hash<btScalar>{}(v.z());

    return h_x^(h_y<<1)^(h_z<<2);
  }
};

template<>
struct std::hash<std::pair<unsigned int, unsigned int>>
{
  std::size_t operator()(const std::pair<unsigned int,unsigned int>& v) const noexcept
  {
    size_t h_first=std::hash<unsigned int>{}(v.first);
    size_t h_second=std::hash<unsigned int>{}(v.second);

    return h_first^(h_second<<1);
  }
};

