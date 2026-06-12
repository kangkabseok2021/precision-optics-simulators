#include "tracer/Camera.h"
#include <cmath>

Camera::Camera(Vec3 origin, Vec3 look_at, Vec3 up, float vfov_deg, float aspect) {
    float theta = vfov_deg * 3.14159265f / 180.0f;
    float h     = std::tan(theta / 2.0f);
    float vp_h  = 2.0f * h;
    float vp_w  = aspect * vp_h;

    Vec3 w = (origin - look_at).normalized();
    Vec3 u = up.cross(w).normalized();
    Vec3 v = w.cross(u);

    origin_     = origin;
    horizontal_ = u * vp_w;
    vertical_   = v * vp_h;
    lower_left_ = origin_ - horizontal_*0.5f - vertical_*0.5f - w;
}

Ray Camera::generate_ray(float u, float v) const noexcept {
    Vec3 dir = (lower_left_ + horizontal_*u + vertical_*v - origin_).normalized();
    return Ray{origin_, dir};
}
