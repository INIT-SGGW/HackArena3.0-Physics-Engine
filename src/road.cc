#include "boink/simulators/track/road.h"

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <LinearMath/btQuaternion.h>

#include "boink/collision_group.h"
#include "boink/exception.h"
#include "boink/constants.h"
#include "boink/assert.h"

#include <fstream>
#include <ios>
#include <random>

namespace boink
{
  Road::Road(
      Line centerline, 
      Line rightline, 
      Line leftline, 
      std::shared_ptr<btDynamicsWorld> world,
      std::string file)
    :centerline_(std::move(centerline)),
     rightline_(std::move(rightline)),
     leftline_(std::move(leftline)),
     world_(world),
     file_(file)
  {
    (void)world;
    if(centerline_.getPointsSize()<3||
        rightline_.getPointsSize()<3||
        leftline_.getPointsSize()<3)
    {
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Center, right or left line is made of less than 3 points");
    }

    // Check if lines should be reveresed
    {
      auto center_point=centerline_.getPoint(0);
      auto next_center_point=centerline_.getPoint(1);

      auto dir=next_center_point-center_point;

      auto right=rightline_.getClosest(center_point)->first-center_point;
      auto normal=right.cross(dir);
      if(normal.dot(g_Up)<0)
      {
        centerline_.reverse();
        dir*=-1;
      }

      auto right_point=rightline_.getPoint(0);
      auto next_right_point=rightline_.getPoint(1);
      auto right_dir=next_right_point-right_point;
      
      if(right_dir.dot(dir)<0.f)
        rightline_.reverse();

      auto left_point=leftline_.getPoint(0);
      auto next_left_point=leftline_.getPoint(1);
      auto left_dir=next_left_point-left_point;
      
      if(left_dir.dot(dir)<0.f)
        leftline_.reverse();
    }

    if(
        !(centerline_.isClosed() && rightline_.isClosed() && leftline_.isClosed()) && 
        !(!centerline_.isClosed() && !rightline_.isClosed() && !leftline_.isClosed()))
    {
      throw Exception(
          Exception::Type::InternalError,
          "All lines must be closed or open");
    }

    is_road_closed_=centerline_.isClosed();

    this->loadMetrics();

    if(road_data_.size()!=centerline_.getPointsSize())
      throw Exception(
          Exception::Type::InternalError,
          "Road data and centerline points sizes do not match");
  }

  const btVector3 Road::getPoint(size_t index,Side side) const
  {
    const Line& line=getLine(side);

    BOINK_ASSERT(index<line.getPointsSize() && index<road_data_.size());

    return line.getPoint(index);
  }

  const btVector3 Road::getInterpolatedPoint1(const btVector3& point,Side side) const
  {
    const Line& line=getLine(side);
    btVector3 interpolated=line.getClosestPointInterpolated1(point).first;

    return interpolated;
  }

  const Road::Metrics& Road::getMetrics(size_t index) const
  {
    BOINK_ASSERT(index<centerline_.getPointsSize() && index<road_data_.size());

    return road_data_.at(index);
  }

  btVector3 Road::getRandomPosition(Side side) const
  {
    const Line& line=getLine(side);

    std::random_device rd;
    static std::mt19937 gen(rd());

    std::uniform_int_distribution<size_t> dist(0,line.getPointsSize()-1);
    size_t random_index=dist(gen);

    BOINK_ASSERT(random_index<road_data_.size());
    if(random_index>=road_data_.size())
      return line.getPoint(random_index);

    const auto& sample=road_data_.at(random_index);

    std::uniform_real_distribution<btScalar> real_dist(0.0f,1.f);
    btScalar random_left_width=real_dist(gen)*sample.left_width;
    btScalar random_right_width=real_dist(gen)*sample.right_width;

    btVector3 offset=sample.right*(random_right_width-random_left_width);
    btVector3 random_point=line.getPoint(random_index);

    return random_point+offset;
  }

  const Road::Metrics& Road::getClosestMetrics(const btVector3& point) const
  {
    size_t index=std::distance(centerline_.begin(), centerline_.getClosest(point));
    
    BOINK_ASSERT(index<road_data_.size());
    if(index>=road_data_.size())
      return road_data_.at(0);;

    const auto& sample=road_data_.at(index);

    return sample;
  }

  int Road::isObjectOnRoad(
      const btVector3& position,
      const btQuaternion& orientation,
      const btVector3& offset,
      const BoundingBox& box,
      bool max_lines) const
  {
    (void) max_lines;

    static bool warn=false;
    if(!warn)
    {
      BOINK_WARN("Max lines boolean not implemented");
      warn=true;
    }

    btVector3 corners[4]={
      offset+box.bottom_left,
      offset+box.bottom_right,
      offset+box.top_left,
      offset+box.top_right
    };

    bool corner_overlaps[4]={
      true,
      true,
      true,
      true
    };

    for(int i=0;i<4;i++)
    {
      btVector3 world_corner=position+quatRotate(orientation,corners[i]);
      btVector3 point_interpolated=getInterpolatedPoint1(world_corner);

      size_t sample_index=
        std::distance(centerline_.begin(), centerline_.getClosest(world_corner));
      const auto& sample=road_data_[sample_index];

      btVector3 rel_pos=world_corner-point_interpolated;
      btScalar lateral_dist=rel_pos.dot(sample.right);

      if(lateral_dist>0)
      {
        if(lateral_dist>sample.right_width)
        {
          corner_overlaps[i]=false;
          continue;
        }
      }
      else
      {
        if(-lateral_dist>sample.left_width)
        {
          corner_overlaps[i]=false;
          continue;
        }
      }

      if(!is_road_closed_)
      {
        if(sample_index==0)
        {
          btVector3 first_point=getPoint(sample_index);
          btVector3 rel_pos_f=world_corner-first_point;
          const auto& sample_first=sample;

          if(rel_pos_f.dot(sample_first.tangent)<0)
          {
            corner_overlaps[i]=false;
            continue;
          }
        }
        else if(sample_index==getSize(Side::Center)-1)
        {
          btVector3 last_point=getPoint(sample_index);
          btVector3 rel_pos_f=world_corner-last_point;
          const auto& last_first=sample;

          if(rel_pos_f.dot(last_first.tangent)>0)
          {
            corner_overlaps[i]=false;
            continue;
          }
        }
      }
    }

    int num_corners_on_road=0;
    for( bool is_corner_on_road:corner_overlaps)
    {
      if(is_corner_on_road)
        num_corners_on_road++;
    }

    return num_corners_on_road;
  }

  void Road::loadMetrics()
  {
    std::ifstream in(file_, std::ios_base::binary | std::ios_base::in);

    if(!in)
    {
      createRoadData();
      saveMetrics();
      return;
    }

    size_t size;
    in.read((char*)&size, sizeof(size));

    std::vector<Metrics> data(size);

    for (auto& m : data)
    {
      in.read((char*)&m.tangent, sizeof(btVector3));
      in.read((char*)&m.normal, sizeof(btVector3));
      in.read((char*)&m.right, sizeof(btVector3));

      in.read((char*)&m.left_width, sizeof(btScalar));
      in.read((char*)&m.left_max_width, sizeof(btScalar));

      size_t leftSize;
      in.read((char*)&leftSize, sizeof(leftSize));
      m.left_grounds.resize(leftSize);
      for (auto& p : m.left_grounds)
      {
          in.read((char*)&p.first, sizeof(btScalar));
          in.read((char*)&p.second, sizeof(Ground::Type));
      }

      in.read((char*)&m.right_width, sizeof(btScalar));
      in.read((char*)&m.right_max_width, sizeof(btScalar));

      size_t rightSize;
      in.read((char*)&rightSize, sizeof(rightSize));
      m.right_grounds.resize(rightSize);
      for (auto& p : m.right_grounds)
      {
          in.read((char*)&p.first, sizeof(btScalar));
          in.read((char*)&p.second, sizeof(Ground::Type));
      }

      in.read((char*)&m.curvature, sizeof(btScalar));
      in.read((char*)&m.grade, sizeof(btScalar));
      in.read((char*)&m.bank, sizeof(btScalar));
    }

    road_data_=std::move(data);
  }

  void Road::saveMetrics()
  {
    std::ofstream out(file_,std::ios_base::binary|std::ios_base::out);

    if(!out)
      throw Exception(
          Exception::Type::IOError,
          "Failed to load track binary data");
    
    size_t size=road_data_.size();
    out.write((char*)(&size),sizeof(size));

    for (const auto& m : road_data_)
    {
      out.write((char*)&m.tangent, sizeof(btVector3));
      out.write((char*)&m.normal, sizeof(btVector3));
      out.write((char*)&m.right, sizeof(btVector3));

      out.write((char*)&m.left_width, sizeof(btScalar));
      out.write((char*)&m.left_max_width, sizeof(btScalar));

      size_t leftSize = m.left_grounds.size();
      out.write((char*)&leftSize, sizeof(leftSize));
      for (auto& p : m.left_grounds)
      {
          out.write((char*)&p.first, sizeof(btScalar));
          out.write((char*)&p.second, sizeof(Ground::Type));
      }

      out.write((char*)&m.right_width, sizeof(btScalar));
      out.write((char*)&m.right_max_width, sizeof(btScalar));

      size_t rightSize = m.right_grounds.size();
      out.write((char*)&rightSize, sizeof(rightSize));
      for (auto& p : m.right_grounds)
      {
          out.write((char*)&p.first, sizeof(btScalar));
          out.write((char*)&p.second, sizeof(Ground::Type));
      }

      out.write((char*)&m.curvature, sizeof(btScalar));
      out.write((char*)&m.grade, sizeof(btScalar));
      out.write((char*)&m.bank, sizeof(btScalar));
    }
  }

  void Road::createRoadData()
  {
    road_data_.reserve(centerline_.getPointsSize());

    BOINK_ASSERT(centerline_.getPointsSize()>2);
    BOINK_ASSERT(rightline_.getPointsSize()>2);
    BOINK_ASSERT(leftline_.getPointsSize()>2);

    for(size_t i=0;i<centerline_.getPointsSize();i++)
      road_data_.push_back(this->generateMetrics(i));

    for(size_t i=0;i<road_data_.size();i++)
      road_data_[i].curvature=calculateCurvature(i);

    if(road_data_.size()!=centerline_.getPointsSize())
      throw Exception(
          Exception::Type::InternalError,
          "After read track_data and center line points sizes does not match");

    if(!is_road_closed_)
    {
      // Edge line can be too short and algorthm can think that the width there is
      // zero
      for(size_t i=0;i<road_data_.size();i++)
      {
        if(road_data_[i].left_width==0)
        {
          btScalar left_width_neighbour=
            i+1<road_data_.size()?
            road_data_[i+1].left_width:
            road_data_[i-1].left_width;

          if(left_width_neighbour==0)
            throw Exception(
                Exception::Type::UnsupportedFormatError,
                "The edge lines for open lines must be longer on the ends than center"
                " line");

          road_data_[i].left_width=left_width_neighbour;
        }

        if(road_data_[i].right_width==0)
        {
          btScalar right_width_neighbour=
            i+1<road_data_.size()?
            road_data_[i+1].right_width:
            road_data_[i-1].right_width;

          if(right_width_neighbour==0)
            throw Exception(
                Exception::Type::UnsupportedFormatError,
                "The edge lines for open lines must be longer on the ends than center"
                " line");

          road_data_[i].right_width=right_width_neighbour;
        }
      }
    }
  }

  Road::Metrics Road::generateMetrics(
      size_t i) const
  {
    Metrics sample;
    size_t centerline_size=centerline_.getPointsSize();

    const auto& [center_point,dist]=centerline_.getPointAndDist(i);
    btVector3 right_point=
      rightline_.getClosest(center_point)->first;

    if(!is_road_closed_ &&i==centerline_size-1)
    {
      sample.tangent=center_point-centerline_.getPoint(i-1);
    }
    else
    {
      size_t next=(i+1)%centerline_size;
      sample.tangent=centerline_.getPoint(next)-center_point;
    }

    sample.tangent.normalize();

    // Create real right vector
    sample.right=right_point-center_point;
    sample.right-=sample.right.dot(sample.tangent)*sample.tangent;
    sample.right.normalize();

    if(btFabs(sample.tangent.dot(sample.right))>g_Epsilon)
    {
      std::stringstream ss;
      ss<<"For centerline point i=("<<i;
      ss<<") the dot product of tangent and right vectors is greater than epsilon";
      throw Exception(
          Exception::Type::InternalError,
          ss.str());
    }

    sample.normal=sample.right.cross(sample.tangent);
    BOINK_ASSERT(sample.normal.length2()>g_Epsilon);
    sample.normal.normalize();

    if(sample.normal.dot(g_Up)<0.0)
    {
      std::stringstream ss;
      ss<<"For centerline point i=("<<i;
      ss<<") the dot product of normal and up vectors is negative";
      throw Exception(
          Exception::Type::InternalError,
          ss.str());
    }

    std::pair<btVector3,btScalar> dummy={{0,0,0},0};
    sample.right_width=rightline_.getRayLineIntersection(
        sample.right,center_point,sample.normal).value_or(dummy).second;

    sample.left_width=leftline_.getRayLineIntersection(
        -1*sample.right,center_point,sample.normal).value_or(dummy).second;

    sample.grade=btAsin(sample.tangent.dot(g_Up));

    btVector3 proj_up=g_Up-g_Up.dot(sample.tangent)*sample.tangent;
    btScalar cos_angle=proj_up.dot(sample.normal);
    btScalar sin_angle=proj_up.dot(sample.right);

    sample.bank=btAtan2(sin_angle,cos_angle);

    btVector3 center_move_up=center_point+sample.normal*1.5f;
    sample.left_max_width=calculateFirstHitDist(center_move_up,-sample.right);
    sample.right_max_width=calculateFirstHitDist(center_move_up,sample.right);

    btVector3 down=-sample.normal;
    sample.right_grounds=calculateGroundWidths(
        sample.right*sample.right_width+center_move_up,
        sample.right*sample.right_max_width+center_move_up,down);
    sample.left_grounds=calculateGroundWidths(
        -sample.right*sample.left_width+center_move_up,
        -sample.right*sample.left_max_width+center_move_up,down);

    return sample;
  }

  btScalar Road::calculateCurvature(size_t i) const
  {
    size_t N=road_data_.size();
    size_t left;
    size_t right;
    if(!is_road_closed_ && i==0)
    {
      left=0;
      right=1;
    }
    else if(!is_road_closed_ && i==N-1)
    {
      left=N-2;
      right=N-1;
    }
    else
    {
      left=(i+road_data_.size()-1)%road_data_.size();
      right=(i+1)%road_data_.size();
    }

    const auto& sample_left=road_data_[left];
    const auto& sample_right=road_data_[right];
    btScalar coverage_left=centerline_.getPointAndDist(left).second;
    btScalar coverage_right=centerline_.getPointAndDist(right).second;

    btVector3 dT=sample_right.tangent-sample_left.tangent;

    btScalar ds=coverage_right-coverage_left;
    if(is_road_closed_ && ds<0.f)
      ds+=getLength();

    if(btFabs(ds)<g_Epsilon)
      return 0;
    btVector3 dTds=dT/ds;

    return dTds.dot(road_data_[i].right);
  }

  btScalar Road::calculateFirstHitDist(
      const btVector3& from,const btVector3& dir, btScalar max_search)const
  {
    btVector3 to=from+dir*max_search;

    btCollisionWorld::ClosestRayResultCallback ray_callback(from,to);
    ray_callback.m_collisionFilterGroup=CollisionGroup::Vehicle;
    ray_callback.m_collisionFilterMask=CollisionGroup::Static;

    world_->rayTest(from,to,ray_callback);

    if(ray_callback.hasHit())
      return from.distance(ray_callback.m_hitPointWorld);
    else
      return max_search;
  }

  std::vector<std::pair<btScalar,Ground::Type>> Road::calculateGroundWidths(
      const btVector3& from,
      const btVector3& to,
      btVector3 down,
      btScalar step) const
  {
    btScalar max_dist=from.distance(to);
    std::vector<std::pair<btScalar, Ground::Type>> grounds;

    btVector3 dir=(to-from).normalized();
    
    btVector3 rayVector=down.normalized()*50.0f;

    Ground::Type prev_type = Ground::Type::Count;

    for (btScalar dist = 0.0f; dist <= max_dist; dist += step)
    {
      // FIX: Calculate point relative to 'from'
      btVector3 currentPoint = from + (dir * dist);

      btCollisionWorld::ClosestRayResultCallback ray_callback(
          currentPoint, currentPoint + rayVector);
      ray_callback.m_collisionFilterGroup = CollisionGroup::Vehicle;
      ray_callback.m_collisionFilterMask = CollisionGroup::Static;

      world_->rayTest(currentPoint, currentPoint + rayVector, ray_callback);

      Ground::Type current_type = Ground::Type::Count; // Default to "No Hit"

      if (ray_callback.hasHit())
      {
        void* ptr = ray_callback.m_collisionObject->getUserPointer();
        if (ptr)
        {
          // Cast safely
          auto* userData = static_cast<Ground::UserData*>(ptr);
          if (userData && userData->p_surface_info)
            current_type = userData->p_surface_info->type;
        }
      }

      // Detect if the surface type changed (e.g., Asphalt -> Grass, or Grass -> None)
      if (current_type != prev_type)
      {
        grounds.emplace_back(dist, current_type);
        prev_type = current_type;
      }
    }

    return grounds;
  }

  Line& Road::getLine(Side side)
  {
    return const_cast<Line&>(const_cast<const Road&>(*this).getLine(side));
  }

  const Line& Road::getLine(Side side) const
  {
    switch(side)
    {
      case Side::Center:
        return centerline_;
      case Side::Right:
        return rightline_;
      case Side::Left:
        return leftline_;
    }

    BOINK_ASSERT(false, "Side is not of any enum "
        "related point falling back to centerline");

    return centerline_;
  }
}
