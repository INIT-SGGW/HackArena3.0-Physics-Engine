#pragma once

#include <LinearMath/btTransform.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btVector3.h>

#include <glm/detail/qualifier.hpp>
#include <glm/mat4x4.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

#include <utility>
#include <vector>

namespace boink
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
}
