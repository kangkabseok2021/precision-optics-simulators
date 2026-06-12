#pragma once
#include "Vec3.h"

struct Ray {
    Vec3  origin;
    Vec3  direction;
    float tmin{1e-4f};
    float tmax{1e30f};

    Vec3 at(float t) const noexcept { return origin + direction * t; }
};
