#pragma once

namespace boink
{
inline btScalar scalarLerp(btScalar from, btScalar to, btScalar ratio) { return from + (to - from) * ratio; }
}  // namespace boink
