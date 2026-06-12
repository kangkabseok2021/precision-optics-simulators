#include "geometry/AABB.h"
#include <algorithm>

bool AABB::intersect(const Ray& ray) const noexcept {
    float tmin = ray.tmin, tmax = ray.tmax;
    for (int i = 0; i < 3; ++i) {
        float inv_d = 1.0f / ray.direction[i];
        float t0 = (min_pt[i] - ray.origin[i]) * inv_d;
        float t1 = (max_pt[i] - ray.origin[i]) * inv_d;
        if (inv_d < 0.0f) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax < tmin) return false;
    }
    return true;
}
