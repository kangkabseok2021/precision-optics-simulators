#pragma once
#include "geometry/Vec3.h"
#include "geometry/Ray.h"

class Camera {
public:
    Camera(Vec3 origin, Vec3 look_at, Vec3 up, float vfov_deg, float aspect);
    Ray generate_ray(float u, float v) const noexcept;

private:
    Vec3  origin_;
    Vec3  lower_left_;
    Vec3  horizontal_;
    Vec3  vertical_;
};
