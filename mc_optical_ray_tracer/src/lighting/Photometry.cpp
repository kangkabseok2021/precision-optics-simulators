#include "lighting/Photometry.h"
#include <cmath>

float compute_illuminance(Vec3 source, Vec3 point, Vec3 normal, float intensity) noexcept {
    Vec3  to_source = source - point;
    float r_sq      = to_source.length_sq();
    if (r_sq < 1e-12f) return 0.0f;
    Vec3  dir       = to_source * (1.0f / std::sqrt(r_sq));
    float cos_theta = dir.dot(normal);
    if (cos_theta <= 0.0f) return 0.0f;
    return intensity * cos_theta / r_sq;
}
