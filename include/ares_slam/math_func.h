#ifndef MATH_FUNC_H
#define MATH_FUNC_H

#include <math.h>

namespace ares_slam {

template <typename T>
inline T square(T value)
{
    return value * value;
}

// Converts from degrees to radians.
inline double degToRad(double deg)
{
    return deg / 180.0 * M_PI;
}

// Converts form radians to degrees.
inline double radToDeg(double rad)
{
    return rad / M_PI * 180.0;
}

// Normalize angle into [-pi; pi].
inline double normalizeAngle(double angle)
{
    while(angle >= M_PI)
    {
        angle -= 2 * M_PI;
    }
    while(angle <= -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

} // namespace ares_slam

#endif // MATH_FUNC_H
