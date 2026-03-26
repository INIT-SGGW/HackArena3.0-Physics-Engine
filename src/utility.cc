#include "boink/utility.h"

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btScalar.h>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

#include "boink/assert.h"

namespace boink::math
{

  glm::mat4 bt2glm(const btTransform& bt_trans)
  {
    return 
      glm::translate(glm::mat4(1.f),bt2glm(bt_trans.getOrigin()))*
      glm::mat4(bt2glm(bt_trans.getBasis()));
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

    assert(scale[0][0]>=0);
    assert(scale[1][1]>=0);
    assert(scale[2][2]>=0);

    glm::mat4 rotation(1.f);
    rotation[0]=transform[0]/scale[0][0];
    rotation[1]=transform[1]/scale[1][1];
    rotation[2]=transform[2]/scale[2][2];

    return {translate,rotation,scale};
  }

  bool areColinear(
      const btVector3& a, const btVector3& b, btScalar epsilon)
  {
    btVector3 cross=a.cross(b);
    return cross.length2()<epsilon;
  }

  size_t getIthClosestIndex(const std::vector<btVector3>& vec,
      const btVector3& point,size_t ith)
  {
    BOINK_ASSERT(ith<=vec.size());
    BOINK_ASSERT(ith>=1);

    std::vector<bool> used(vec.size(),false);
    size_t i_closest = 0;
    for(size_t i=0;i<ith;i++)
    {
      btScalar i_length2=BT_LARGE_FLOAT;
      for(size_t j=0;j<vec.size();j++)
      {
        if(used[j])
          continue;

        btScalar length2=(point-vec[j]).length2();
        if(length2<i_length2)
        {
          i_closest=j;
          i_length2=length2;
        }
      }
      used[i_closest]=true;
    }

    return i_closest;
  }

  btVector3 vec3toLocalvec2(
      const btVector3& vec,
      const btVector3& local_x,
      const btVector3& local_y,
      const btVector3& local_origin)
  {
    btVector3 offset=vec-local_origin;
    return btVector3(local_x.dot(offset),local_y.dot(offset),0.f);
  }

  std::optional<std::tuple<btVector3,btScalar,btScalar>> getRayLineInterscetion(
      btVector3 ray_dir,
      btVector3 ray_start,
      btVector3 normal,
      btVector3 a,
      btVector3 b,
      btScalar epsilon)
  {
    normal.normalize();
    ray_dir.normalize();

    // Calculate plane
    btVector3 arbitrary_vec=
      btFabs(normal.y())>0.9?
      btVector3{1.f,0.f,0.f}:btVector3{0.f,1.f,0.f};
    
    btVector3 local_x=normal.cross(arbitrary_vec).normalize();
    btVector3 local_y=normal.cross(local_x).normalize();
    
    // Turn vec3 to vec2 in local plane
    btVector3 local_ray_dir(ray_dir.dot(local_x),ray_dir.dot(local_y),0.f);
    BOINK_ASSERT(local_ray_dir.length()>1.f-epsilon &&
        local_ray_dir.length()<1.f+epsilon);

    if(local_ray_dir.length2() <epsilon*epsilon)
      return std::nullopt;

    btVector3 local_ray_start(vec3toLocalvec2(ray_start,local_x,local_y,ray_start));
    btVector3 local_a(vec3toLocalvec2(a,local_x,local_y,ray_start));
    btVector3 local_b(vec3toLocalvec2(b,local_x,local_y,ray_start));

    btVector3 c=local_b-local_a;
    btVector3 v=local_ray_start-local_a;

    btScalar cross_2d=local_ray_dir.x()*c.y()-local_ray_dir.y()*c.x();

    // Means line is parallel to line
    if(btFabs(cross_2d)<epsilon)
      return std::nullopt;

    btScalar t=(c.x()*v.y()-c.y()*v.x())/cross_2d;
    btScalar u=(local_ray_dir.x()*v.y()-local_ray_dir.y()*v.x())/cross_2d;

    btVector3 intersection_point=
      a+u*(b-a);
    
    return {{intersection_point,t,u}};
  }
}
