#include "boink/utils/utility.h"

#include <LinearMath/btQuaternion.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace boink
{

  glm::mat4 bt2glm(const btTransform& bt_trans)
  {
    // Rotation
    glm::mat4 glm_mat(
        bt2glm(bt_trans.getBasis()));
    
    return glm::translate(glm_mat,bt2glm(bt_trans.getOrigin()));
  }

  glm::mat3 bt2glm(const btMatrix3x3& bt_mat)
  {
    return glm::mat3(
        bt2glm(bt_mat.getColumn(0)),
        bt2glm(bt_mat.getColumn(1)),
        bt2glm(bt_mat.getColumn(2)));
  }

  glm::vec3 bt2glm(const btVector3& bt_vec)
  {
    return glm::vec3(
      bt_vec.getX(),
      bt_vec.getY(),
      bt_vec.getZ());
  }

  std::pair<btVector3,btTransform> glm2bt(const glm::mat4& glm_mat)
  {
    // DO NOT apply scale to btTransform it is supposed to be
    // done using btCollisonShape::setLocalScaling()

    btVector3 translate(glm2bt(glm::vec3(glm_mat[3])));
    btVector3 scale(
        glm::length(glm_mat[0]),
        glm::length(glm_mat[1]),
        glm::length(glm_mat[2]));
    glm::mat3 glm_rot(
        glm_mat[0]/scale.getX(),
        glm_mat[1]/scale.getY(),
        glm_mat[2]/scale.getZ());
    glm::quat glm_quat=glm::quat_cast(glm_rot);
    btQuaternion bt_quat(
        glm_quat.x,
        glm_quat.y,
        glm_quat.z,
        glm_quat.w);

    btTransform transform(bt_quat,translate);
    return {scale,transform};
  }

  btMatrix3x3 glm2bt(const glm::mat3& glm_mat)
  {
    return btMatrix3x3(
        glm_mat[0][0],glm_mat[1][0],glm_mat[2][0],
        glm_mat[0][1],glm_mat[1][1],glm_mat[2][1],
        glm_mat[0][2],glm_mat[1][2],glm_mat[2][2]);
  }

  btVector3 glm2bt(const glm::vec3& glm_vec)
  {
    return btVector3(
        glm_vec.x,
        glm_vec.y,
        glm_vec.z);
  }

  std::tuple<glm::mat4,glm::mat4,glm::mat4> 
  decomposeMatrix(const glm::mat4& transform)
  {
    glm::mat4 translate=
      glm::translate(glm::mat4(1.f),glm::vec3(transform[3]));
    
    glm::mat4 scale(1.f);
    scale[0][0]=glm::length(transform[0]);
    scale[1][1]=glm::length(transform[1]);
    scale[2][2]=glm::length(transform[2]);

    glm::mat4 rotation(1.f);
    rotation[0]=transform[0]/scale[0][0];
    rotation[1]=transform[1]/scale[1][1];
    rotation[2]=transform[2]/scale[2][2];

    return {translate,rotation,scale};
  }
}
