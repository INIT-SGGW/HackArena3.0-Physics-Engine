#include "boink/simulators/track/line.h"

#include <LinearMath/btScalar.h>
#include <algorithm>
#include <cassert>
#include <optional>
#include <unordered_set>

#include "boink/constants.h"
#include "boink/exception.h"
#include "boink/utility.h"

namespace boink {

Line::Line(
    const btVector3& first_point,
    const std::vector<btVector3>& points,
    bool is_line_closed) 
  : 
    points_dist_(points.size()),
    is_line_closed_(is_line_closed)
{
  if (points.size() < 2)
    throw Exception(Exception::Type::InvalidArgumentError,
                    "The vector of given points is lesser than 2");

  assert(points_dist_.size() == points.size());
  std::transform(points.cbegin(), points.cend(), points_dist_.begin(),
                 [](const btVector3& v) {
                   return std::pair<btVector3, btScalar>(v, 0.0f);
                 });

  // Get index of the closest point to first point.
  size_t first_point_index=0;
  btScalar min_dist=FLT_MAX;
  for(size_t i=0;i<points.size();i++)
  {
    btScalar dist=btFabs((points[i]-first_point).length2());
    if(dist<min_dist)
    {
      first_point_index=i;
      min_dist=dist;
    }
  }

  // Order the points
  // We assume that first given point is the
  // starting point. fuck for now direciton
  std::vector<size_t> sorted_indices;
  std::vector<size_t> unused_indices(points_dist_.size());

  std::generate(unused_indices.begin(), unused_indices.end(),
                [n = 0]() mutable { return n++; });

  // Assume first element of points is the first in order
  sorted_indices.push_back(first_point_index);
  unused_indices.erase(
      std::find(unused_indices.begin(), unused_indices.end(), first_point_index));

  // Very inefficient
  while (unused_indices.size() > 0) {
    size_t prev_i = sorted_indices[sorted_indices.size() - 1];
    size_t curr_i = this->getIthClosestIndex(this->getPoint(prev_i), 2).first;
    size_t ith = 3;
    while (std::find(unused_indices.begin(), unused_indices.end(), curr_i) ==
           unused_indices.end()) {
      curr_i = this->getIthClosestIndex(this->getPoint(prev_i), ith).first;
      ith++;
    }

    sorted_indices.push_back(curr_i);
    unused_indices.erase(
        std::find(unused_indices.begin(), unused_indices.end(), curr_i));
  }

  assert(sorted_indices.size() == points_dist_.size());
  assert(sorted_indices.size() == points.size());

  std::unordered_set<size_t> seen;
  btScalar length = 0.0;
  size_t prev_index = sorted_indices[0];
  for (size_t i = 0; i < sorted_indices.size(); i++) {
    size_t index = sorted_indices[i];
    if (seen.insert(index).second) {
      length += (points[index] - points[prev_index]).length();

      //assert(length > 1e-6 || i == 0);
      points_dist_[i] = {points[index], length};
      prev_index = index;
    }
  }

  // Delete duplicated entries in point_dist_
  // they are sorted so we look for entries that are next to each other
  for (auto it = points_dist_.begin(); it + 1 != points_dist_.end();) {
    auto it2 = it + 1;
    if (btFabs(it->second - it2->second) < 1e-8)
      it = points_dist_.erase(it);
    else
      it++;
  }
}

btScalar Line::getLength() const {
  btScalar dist=points_dist_[points_dist_.size() - 1].second;
  if(is_line_closed_)
    dist+=
      (points_dist_[points_dist_.size()-1].first-points_dist_[0].first).length();

  return dist;
}

btScalar Line::getCoverage(const btVector3& point) const {
  size_t closest_i = this->getClosestIndex(point).first;
  size_t sec_closest_i = this->getIthClosestIndex(point, 2).first;

  size_t first_i;
  size_t second_i;
  if (points_dist_[closest_i].second < points_dist_[sec_closest_i].second) {
    first_i = closest_i;
    second_i = sec_closest_i;
  } else {
    second_i = closest_i;
    first_i = sec_closest_i;
  }

  // if are the first and last points
  if(is_line_closed_)
  {
    if (first_i == 0 && second_i == this->getPointsSize() - 1) {
      first_i = second_i;
      second_i = 0;
    }
  }
  assert(first_i != second_i);

  btVector3 dir = points_dist_[second_i].first - points_dist_[first_i].first;
  btScalar len = dir.length();
  if (len < 0.0001)
    dir = {0, 0, 0};
  else
    dir.normalize();
  btVector3 our_vec = point - points_dist_[first_i].first;

  // assert(our_vec.dot(dir)>0);
  return our_vec.dot(dir) + points_dist_[first_i].second;
}

std::pair<size_t,btScalar> Line::getClosestIndex(
      const btVector3& point) const
{
  size_t closest_i=0;
  btScalar closest_dist2=(points_dist_[closest_i].first-point).length2();
  for(size_t i=1;i<points_dist_.size();i++)
  {
    btScalar curr_len2=(points_dist_[i].first-point).length2();
    if(curr_len2<closest_dist2)
    {
      closest_dist2=curr_len2;
      closest_i=i;
    }
  }

  return {closest_i,btSqrt(closest_dist2)};
}


std::pair<size_t,btScalar> Line::getIthClosestIndex(
    const btVector3& point, size_t ith) const
{
  assert(points_dist_.size()>ith);

  std::vector<size_t> closest_is;
  btScalar ith_closest_dist2=0;
  size_t ith_closest_i;
  for(size_t i=0;i<ith;i++)
  {
    ith_closest_i=0;
    while(std::find(closest_is.begin(),closest_is.end(),ith_closest_i)
        !=closest_is.end())
    {
      ith_closest_i++;
      assert(ith_closest_i<points_dist_.size());
    }
    ith_closest_dist2=(this->getPoint(ith_closest_i)-point).length2();
    
    for(size_t j=0;j<points_dist_.size();j++)
    {
      if(std::find(closest_is.begin(),closest_is.end(),j)!=closest_is.end())
      {
        continue;
      }

      btScalar curr_len2=(this->getPoint(j)-point).length2();
      if(curr_len2<ith_closest_dist2)
      {
        ith_closest_dist2=curr_len2;
        ith_closest_i=j;
      }
    }
    closest_is.push_back(ith_closest_i);
  }

  return {ith_closest_i,btSqrt(ith_closest_dist2)};
}


std::optional<btVector3> Line::getClosestPointInterpolated(
    const btVector3& point) const
{
  auto [i_closest,_]=this->getClosestIndex(point);
  const btVector3& closest=this->getPoint(i_closest);

  // We now check if i_closet+-1 is the correct one.
  for( int index : {-1,1})
  {
    int i_other=static_cast<int>(i_closest)+index;
    if((i_other <0 || i_other >=(int)this->getPointsSize())&& !is_line_closed_)
      continue;

    i_other%=this->getPointsSize();
    const btVector3& other=this->getPoint(i_other);

    if((other-closest).length2()>g_Epsilon)
    {
      btVector3 interpolated_point=Line::getPointInterpolated(
          other,
          closest,
          point);

      if((other-interpolated_point).dot(closest-interpolated_point)<0)
        return interpolated_point;
    }
  }

  if(is_line_closed_)
    btAssert(false && "Line points data are incorretly imported");

  return std::nullopt;
}

btVector3 Line::getPointInterpolated(
    const btVector3& a,
    const btVector3& b,
    const btVector3& point)
{
  btVector3 ab=b-a;
  btScalar ab_len2=ab.length2();

  btAssert(ab_len2>g_Epsilon);
  if(ab_len2<g_Epsilon)
    return a;

  btScalar t=(point-a).dot(ab)/ab_len2;

  return a+t*ab;
}

void Line::reverse()
{
  std::reverse(points_dist_.begin(),points_dist_.end());
  auto last_data=points_dist_[points_dist_.size()-1];
  points_dist_.pop_back();
  points_dist_.insert(points_dist_.begin(),last_data);

  // and know we neeed to update distance :(
  btScalar lenght=0.0f;
  btVector3 prev=points_dist_[0].first;
  for(size_t i=0;i<points_dist_.size();i++)
  {
    lenght+=(points_dist_[i].first-prev).length();
    points_dist_[i].second=lenght;

    prev=points_dist_[i].first;
  }
}

btVector3& Line::getPoint(size_t index)
{
  btAssert(index<points_dist_.size());

  return points_dist_[index].first;
}

const btVector3& Line::getPoint(size_t index) const
{
  return points_dist_.at(index).first;
}

std::pair<btVector3,btScalar> Line::getRayLineIntersection(
    btVector3 ray_dir,
    btVector3 ray_start,
    btVector3 normal,
    btScalar epsilon) const
{
  btScalar t_closest=FLT_MAX;
  btVector3 point_closest;
  for(size_t i=0;i<points_dist_.size()-1;i++)
  {
    auto ray_info=math::getRayLineInterscetion(
        ray_dir,
        ray_start,
        normal,
        points_dist_[i].first,
        points_dist_[i+1].first,
        epsilon);

    if(!ray_info.has_value())
      continue;

    auto [point,t,u]=ray_info.value();

    if(u<0.f||u>1.f)
      continue;

    if(t<0.f)
      continue;

    if(t<t_closest)
    {
      point_closest=point;
      t_closest=t;
    }
  }

  if(is_line_closed_)
  {
    auto ray_info=math::getRayLineInterscetion(
        ray_dir,
        ray_start,
        normal,
        points_dist_[points_dist_.size()-1].first,
        points_dist_[0].first,
        epsilon);

    if(!ray_info.has_value())
      return {point_closest,t_closest};

    auto [point,t,u]=ray_info.value();

    if(u<0.f||u>1.f)
      return {point_closest,t_closest};

    if(t<0.f)
      return {point_closest,t_closest};

    if(t<t_closest)
    {
      point_closest=point;
      t_closest=t;
    }
  }

  return {point_closest,t_closest};

}

Line Line::createLine(
    const GltfExtractor& extractor, 
    const btVector3& first_point,
    std::string_view name)
{
  auto& line_node=extractor.getNode(name);
  if(line_node.type!=TINYGLTF_MODE_LINE)
    throw Exception(
        Exception::Type::UnsupportedFormatError,
        "Line mesh unsupported mode. Use lines mode for line mesh.");

  auto& line_vertices=line_node.vertices;
  auto& line_indices=line_node.indices;
  if(line_vertices.size()==0 || line_indices.size()==0)
    throw Exception(
        Exception::Type::InvalidArgumentError,
        "Line mesh is empty.");

  std::vector<btVector3> points;
  points.reserve(line_indices.size());

  for(size_t i=0;i<line_indices.size();i++)
    points.push_back(line_vertices[line_indices[i]]);

  // Add last point
  //points.push_back(
  //    line_vertices[line_indices[line_indices.size()-1]]);
  
  return Line(first_point,std::move(points));
}
}  // namespace boink
