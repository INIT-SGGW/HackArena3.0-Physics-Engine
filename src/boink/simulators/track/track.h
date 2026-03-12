#pragma once

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include "boink/gltf_extractor.h"
#include "boink/simulators/simulator.h"
#include "boink/simulators/track/line.h"
#include "boink/simulators/track/ground.h"
#include "boink/simulators/weather.h"

#include <memory>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <optional>

namespace boink
{
  class TrackGui;
  class Track : public Simulator
  {
  public:
    friend class TrackGui;
  public:
    struct SampleData
    {
      btScalar coverage;
      btVector3 position;

      btVector3 tangent;
      btVector3 normal;
      btVector3 right;

      btScalar left_width;
      btScalar right_width;

      btScalar curvature;
      // minus downhill postivie uphill -90 to +90
      btScalar grade;
      btScalar bank;
    };
  public:
    Track(
        std::string_view filename,
        std::shared_ptr<const Weather> weather,
        std::shared_ptr<btDiscreteDynamicsWorld> world);
    Track(const Track&)=delete;
    Track(Track&&)=delete;
    Track& operator=(Track&&)=delete;
    Track& operator=(const Track&)=delete;

    void update(btScalar dt) override;
    void updateRender(Renderer* p_renderer) override;
    std::shared_ptr<piksel::GuiObject> getGui() override;

    const btTransform& getWorldTransform() const {return transform_;}

    const Line& getCenterline() const {return centerline_;}
    std::string_view getFilename() const { return filename_;}

    const std::vector<SampleData>& getTrackData() const {return track_data_;}
    std::vector<SampleData>& getTrackData() {return track_data_;}

    size_t getNumberOfStartingPositions() const {return start_postions_.size();}
    btVector3 getStartingPosition(size_t position) const;
    btVector3 getFinishLine() const {return finish_line_;}

    btVector3 getOnTrackRandomPosition() const;

    btVector3 getForwardDirection(const btVector3& point) const;
  private:
    void initSurfaceInfos();
    void initGrounds(const GltfExtractor& extractor);
    void createLines(const GltfExtractor& extractor);
    void initPositions(const GltfExtractor& extractor);
    void initFinishLine(const GltfExtractor& extractor);

    void createTrackData();
    SampleData generateSampleTrackData(size_t i) const;

  private:
    static std::optional<Ground::Type> resolveGroundTypeFromName(std::string name);
  private:
    //static constexpr std::string_view TRACK_NAME="Asphalt";
    static constexpr std::string_view RIGHTLINE_NAME="LINE_RIGHT";
    static constexpr std::string_view LEFTLINE_NAME="LINE_LEFT";
    static constexpr std::string_view CENTERLINE_NAME="LINE_CENTER";

    static constexpr std::string_view DELIM="_";
    static constexpr std::string_view POSITION_SEG_NAME="POSITION";
    static constexpr std::string_view FINISH_LANE_NAME="FINISH_LINE_POINT";
  private:
    std::vector<Ground> grounds;
    std::shared_ptr<btDiscreteDynamicsWorld> world_;

    std::unordered_map<Ground::Type,Ground::SurfaceInfo> surface_infos_;

    std::vector<btVector3> start_postions_;
    btVector3 finish_line_;

    Line centerline_;
    Line rightline_;
    Line leftline_;

    Line pitstop_centerline_;
    Line pitstop_rightline_;
    Line pitstop_leftline_;

    std::vector<SampleData> track_data_;

    btTransform transform_=btTransform::getIdentity();
    std::string filename_;

    std::shared_ptr<const Weather> weather_;

    bool enable_track_data_vec_draw_=false;
    std::shared_ptr<TrackGui> gui_;
  };
}
