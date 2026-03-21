#pragma once

namespace boink
{
inline btScalar scalarLerp(btScalar a, btScalar b, btScalar t) { return a + (b - a) * t; }
}  // namespace boink
