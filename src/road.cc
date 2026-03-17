#include "boink/simulators/track/road.h"

#include "boink/exception.h"
#include "boink/constants.h"
#include "boink/assert.h"

#include <random>

namespace boink
{
  Road::Road(Line centerline, Line rightline, Line leftline)
    :centerline_(std::move(centerline)),
     rightline_(std::move(rightline)),
     leftline_(std::move(leftline))
  {
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

    this->createRoadData();

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

  const btVector3 Road::getInterpolatedPoint(const btVector3& point,Side side) const
  {
    const Line& line=getLine(side);
    btVector3 interpolated=line.getClosestPointInterpolated(point).first;

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
  }

  Road::Metrics Road::generateMetrics(size_t i) const
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

    if(sample.tangent.dot(sample.right)>g_Epsilon)
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

    sample.right_width=(rightline_.getClosestPointInterpolated(
        center_point).first-center_point).length();
    sample.left_width=(leftline_.getClosestPointInterpolated(
        center_point).first-center_point).length();

    sample.grade=btAsin(sample.tangent.dot(g_Up));

    btVector3 proj_up=g_Up-g_Up.dot(sample.tangent)*sample.tangent;
    btScalar cos_angle=proj_up.dot(sample.normal);
    btScalar sin_angle=proj_up.dot(sample.right);

    sample.bank=btAtan2(sin_angle,cos_angle);

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
