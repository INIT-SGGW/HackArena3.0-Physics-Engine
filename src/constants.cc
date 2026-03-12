#include "boink/constants.h"
#include <LinearMath/btVector3.h>

namespace boink
{
  const btVector3 g_Up(0.f,1.f,0.f);
  const btVector3 g_Left(1.f,0.f,0.f);
  const btVector3 g_Forward(0.f,0.f,1.f);

  const int g_MaxSubSteps=15;
  const btScalar g_FixedDeltaTime=1.f/120.f;
  const btScalar g_MaxDeltaTime=0.1f;
}
