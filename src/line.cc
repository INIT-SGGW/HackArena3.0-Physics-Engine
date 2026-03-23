#include "boink/simulators/track/line.h"

#include <LinearMath/btScalar.h>

#include "boink/constants.h"
#include "boink/exception.h"
#include "boink/logger.h"
#include "boink/utility.h"
#include "boink/assert.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <optional>
#include <unordered_set>
#include <vector>
#include <deque>

namespace boink {

Line::Line(
    std::vector<btVector3> points,
    bool is_line_closed) 
  : 
    points_dist_(),
    is_line_closed_(is_line_closed)
{
  if (points.size() < 2)
    throw Exception(Exception::Type::InvalidArgumentError,
                    "The vector of given points is lesser than 2");
  if(is_line_closed_ && points.size() < 3)
    throw Exception(Exception::Type::InvalidArgumentError,
                    "The vector of given points is lesser than 2");

  points_dist_.reserve(points.size());

  // Insert data
  btScalar length=0.f;
  btVector3 prev_point=points[0];
  for(size_t i=0; i<points.size(); i++)
  {
    const btVector3& point=points[i];

    btScalar diff=(point-prev_point).length();
    length+=diff;

    if(diff<g_Epsilon && i!=0)
      BOINK_WARN(
          "Distance between points smaller than epsilon. Index: {}", i);
    if(diff>kDesiredDistance)
      BOINK_TRACE(
          "Distance between points greater than "
          "kDesiredDistance={} [m [m]]. Index: {}",kDesiredDistance, i);

    points_dist_.emplace_back(point, length);
    prev_point = point;
  }

  // Calculate line length
  {
    const auto& point_last=points_dist_.back();
    btScalar dist=point_last.second;
    if(is_line_closed_)
    {
      const auto& point_first=points_dist_.front();
      dist+=(point_last.first-point_first.first).length();
    }

    line_length_=dist;
  }
  BOINK_TRACE("Line length: {}",line_length_);

  for(size_t i=0;i<points_dist_.size();i++)
  {
    const auto& pair=points_dist_[i];
    BOINK_TRACE("i=({}) vec: {} dist: {}",i,pair.first,pair.second);
  }
}

btScalar Line::getCoverage(const btVector3& point) const {
  return this->getClosestPointInterpolated1(point).second;
}

std::vector<std::pair<btVector3,btScalar>>::const_iterator Line::getClosest(
      const btVector3& point) const
{
  // if # of elements exeeds 1e5 cosider using KD-tree
  auto it=std::min_element(points_dist_.cbegin(),points_dist_.cend(),
      [&](const auto& p0, const auto& p1)
      {
        btScalar p0_len2=(p0.first-point).length2();
        btScalar p1_len2=(p1.first-point).length2();
        return p0_len2<p1_len2;
      });

  return it;
}

std::vector<std::pair<btVector3,btScalar>>::const_iterator Line::getIthClosest(
    const btVector3& point, size_t ith) const
{
  if(ith>points_dist_.size() || ith==0)
    return points_dist_.cend();

  auto it=this->getClosest(point);
  size_t index=std::distance(points_dist_.cbegin(),it);

  size_t i_lower=(index+points_dist_.size()-1)%points_dist_.size();
  size_t i_higher=(index+1)%points_dist_.size();
  for(size_t n=1;n<ith;n++)
  {
    btScalar lower_dist2=(getPoint(i_lower)-point).length2();
    btScalar higher_dist2=(getPoint(i_higher)-point).length2();

    if(lower_dist2<higher_dist2)
    {
      index=i_lower;
      i_lower=(i_lower+points_dist_.size()-1)%points_dist_.size();
    }
    else
    {
      index=i_higher;
      i_higher=(i_higher+1)%points_dist_.size();
    }
  }

  return points_dist_.begin()+index;
}


std::pair<btVector3,btScalar> Line::getClosestPointInterpolated1(
    const btVector3& point) const
{
  size_t i_closest=std::distance(points_dist_.begin(),getClosest(point));
  const btVector3& closest=this->getPoint(i_closest);

  // We now check if i_closet+-1 is the correct one.
  for( int index : {-1,1})
  {
    int i_other=static_cast<int>(i_closest)+index;
    if((i_other <0 || i_other >=(int)this->getPointsSize())&& !is_line_closed_)
      continue;

    i_other = (i_other + this->getPointsSize()) % this->getPointsSize();
    const btVector3& other=this->getPoint(i_other);

    btVector3 diff=other-closest;
    if(diff.length2()>g_Epsilon)
    {
      btVector3 interpolated_point=Line::getPointInterpolated(
          other,
          closest,
          point);

      if((other-interpolated_point).dot(closest-interpolated_point)<=0)
      {
        btScalar d_closest=points_dist_[i_closest].second;
        btScalar d_other=points_dist_[i_other].second;

        if(i_closest==0 && i_other==(int)getPointsSize()-1)
          d_closest=getLength();
        else if(i_other==0 && i_closest==getPointsSize()-1)
          d_other=getLength();

        btScalar t=(interpolated_point-closest).dot(diff)/diff.length2();

        btScalar dist=d_closest + t*(d_other-d_closest);
        return {interpolated_point,dist};
      }
    }
  }


  // fallback for open lines or unexpected cases
  // and sharp turns when point is on the outside
  return {closest, points_dist_[i_closest].second};
}

btVector3 Line::getPointInterpolated(
    const btVector3& a,
    const btVector3& b,
    const btVector3& point)
{
  btVector3 ab=b-a;
  btScalar ab_len2=ab.length2();

  BOINK_ASSERT(ab_len2>g_Epsilon);
  if(ab_len2<g_Epsilon)
    return a;

  btScalar t=(point-a).dot(ab)/ab_len2;

  return a+t*ab;
}

void Line::reverse()
{
  std::reverse(points_dist_.begin(),points_dist_.end());
  
  if(isClosed())
  {
    auto last_data=points_dist_[points_dist_.size()-1];
    points_dist_.pop_back();
    points_dist_.insert(points_dist_.begin(),last_data);
  }

  // and now we neeed to update distance :(
  btScalar length=0.0f;
  btVector3 prev=points_dist_[0].first;
  for(size_t i=0;i<points_dist_.size();i++)
  {
    length+=(points_dist_[i].first-prev).length();
    points_dist_[i].second=length;

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
  btAssert(index<points_dist_.size());
  return points_dist_.at(index).first;
}

std::optional<std::pair<btVector3,btScalar>> Line::getRayLineIntersection(
    btVector3 ray_dir,
    btVector3 ray_start,
    btVector3 normal) const
{
  size_t N=this->getPointsSize();
  
  if (N<2)
    return std::nullopt;

  btScalar t_closest=FLT_MAX;
  btVector3 point_closest;
  bool found=false;

  size_t num_segments=is_line_closed_?N:(N-1);

  for (size_t i=0; i<num_segments;++i)
  {
    size_t idx0=i;
    size_t idx1=(i+1)%N;

    auto ray_info = math::getRayLineInterscetion(
        ray_dir,
        ray_start,
        normal,
        points_dist_[idx0].first,
        points_dist_[idx1].first,
        g_Epsilon);

    if (!ray_info.has_value())
      continue;

    auto[point,t,u] = ray_info.value();

    if (u<0.f || u>1.f)
      continue;

    if (t<0.f)
      continue;

    if (t<t_closest)
    {
      point_closest = point;
      t_closest = t;
      found = true;
    }
  }

  if (!found)
    return std::nullopt;

  return {{point_closest,t_closest}};
}

Line Line::createLine(
    const GltfExtractor& extractor, 
    std::string_view name,
    std::optional<btVector3> opt_first_point)
{
  auto& line_node=extractor.getNode(name);
  if(line_node.type!=TINYGLTF_MODE_LINE)
    throw Exception(
        Exception::Type::UnsupportedFormatError,
        "Line mesh unsupported mode. Use lines mode for line mesh.");

  const auto& vertices=line_node.vertices;
  const auto& indices=line_node.indices;
  if(vertices.size()==0 || indices.size()==0)
    throw Exception(
        Exception::Type::InvalidArgumentError,
        "Line mesh is empty.");

  // Generate line strip 
  std::deque<unsigned int> strip;
  {
    // Convert to vertices to line pairs
    std::unordered_set<std::pair<unsigned int,unsigned int>,SegmentHash> 
      segments;
    for(size_t i=0;i+1<indices.size();i+=2)
    {
      unsigned int a=indices[i];
      unsigned int b=indices[i+1];

      // Remove A-A, A-A
      if(a!=b)
      {
        // Remove A--B, B--A
        auto cannonical=std::make_pair(std::min(a,b),std::max(a,b));
        segments.insert(std::move(cannonical));
      }
    }

    if(segments.empty())
      return Line(std::vector<btVector3>(),false);

    strip.push_back(segments.begin()->first);
    strip.push_back(segments.begin()->second);
    segments.erase(segments.begin());

    bool progress=true;
    while(!segments.empty() && progress)
    {
      progress=false;
      for(auto it=segments.begin();it!=segments.end();)
      {
        unsigned int a=it->first;
        unsigned int b=it->second;

        unsigned int first=strip.front();
        unsigned int last=strip.back();
        
        // Means the line is closed
        if(first==last)
          break;
        
        if(a==first)
        {
          strip.push_front(b);
          it=segments.erase(it);
          progress=true;
        }
        else if(a==last)
        {
          strip.push_back(b);
          it=segments.erase(it);
          progress=true;
        }
        else if(b==first)
        {
          strip.push_front(a);
          it=segments.erase(it);
          progress=true;
        }
        else if(b==last)
        {
          strip.push_back(a);
          it=segments.erase(it);
          progress=true;
        }
        else
          ++it;
      }
    }

    if(!segments.empty())
      BOINK_WARN("Not all line segments were used to construct line."
          "Segments left: {}",segments.size());
  }

  bool is_line_closed=(strip.back()==strip.front() && strip.size()>2);
  if(is_line_closed)
    strip.pop_back();

  btVector3 first_point;
  if(!opt_first_point.has_value())
    first_point=vertices[strip.front()];
  else
    first_point=opt_first_point.value();

  auto it=std::min_element(strip.begin(),strip.end(),
      [&](unsigned int idx_a,unsigned int idx_b)
      {
        btScalar len_a2=(vertices[idx_a]-first_point).length2();
        btScalar len_b2=(vertices[idx_b]-first_point).length2();

        return len_a2<len_b2;
      }
  );

  if(it==strip.end())
    throw Exception(
        Exception::Type::UnsupportedFormatError,
        "Line data provided resulted in empty strip");

  size_t first_point_index=std::distance(strip.begin(),it);
  if(it==std::prev(strip.end()))
    std::reverse(strip.begin(),strip.end());
  else
  {
    if(first_point_index!=0&& !is_line_closed)
    {
      BOINK_WARN(
          "Node name: {}. "
          "First point is in the middle of an open line."
          " Falling back to closed line."
          " Vector {} ",name,first_point);

      is_line_closed=true;
    }

    if(it!=strip.begin())
      std::rotate(strip.begin(),it,strip.end());
  }

  // Verify that indexes are unique
  {
    std::unordered_set<unsigned int> unique_check(strip.begin(),strip.end());
    if(unique_check.size()!=strip.size())
    {
      BOINK_WARN("Node name: {}. Strip contains duplicate indices!"
          " Expected: {}, got {} unique",name,strip.size(),unique_check.size());
    }
  }

  std::vector<btVector3> line_points;
  line_points.reserve(strip.size());

  for(unsigned int idx:strip)
    line_points.push_back(vertices[idx]);
  
  return Line(std::move(line_points),is_line_closed);
}
}  // namespace boink
