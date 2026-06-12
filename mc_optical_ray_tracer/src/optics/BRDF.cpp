#include "optics/BRDF.h"
#include <cmath>

Vec3 cosine_sample_hemisphere(float xi1, float xi2, float& pdf) noexcept {
    float phi       = 2.0f * kPi * xi1;
    float cos_theta = std::sqrt(xi2);
    float sin_theta = std::sqrt(1.0f - xi2);
    Vec3 dir{sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
    pdf = cos_theta / kPi;
    return dir;
}

Vec3 lambertian_f(const Material& mat) noexcept {
    return mat.albedo * (1.0f / kPi);
}

Vec3 reflect(const Vec3& wi, const Vec3& normal) noexcept {
    return wi - normal * (2.0f * wi.dot(normal));
}
