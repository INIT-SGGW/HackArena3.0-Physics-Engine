#include "boink/gltf_extractor.h"
#include "boink/debug_drawer.h"

using namespace boink;
void drawCenterline(DebugDrawer& dbg, const GltfExtractor::Node& node);
int main()
{
  boink::GltfExtractor extractor("lowpoly_track_1_test_5.glb");

  const auto& node=extractor.getNode("Centerline");
  DebugDrawer dbg;
  
  while(dbg)
  {
    dbg.drawFrameOrigin();
    drawCenterline(dbg,node);
    dbg.update();
  }
}

void drawCenterline(DebugDrawer& dbg, const GltfExtractor::Node& node)
{
  const auto& vertices=node.vertices;
  const auto& indices=node.indices;

  for(int i=0;i<indices.size();i+=2)
  {
    dbg.drawLine(vertices[indices[i]],vertices[indices[i+1]],{0,1,0});
  }
}
