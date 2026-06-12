#include "geometry/Triangle.h"
#include <cmath>

std::optional<HitRecord> Triangle::intersect(const Ray& ray) const noexcept {
    constexpr float kEpsilon = 1e-8f;
    Vec3 e1 = v1 - v0;
    Vec3 e2 = v2 - v0;
    Vec3 h  = ray.direction.cross(e2);
    float a = e1.dot(h);
    if (std::abs(a) < kEpsilon) return std::nullopt;

    float f = 1.0f / a;
    Vec3  s = ray.origin - v0;
    float u = f * s.dot(h);
    if (u < 0.0f || u > 1.0f) return std::nullopt;

    Vec3  q = s.cross(e1);
    float v = f * ray.direction.dot(q);
    if (v < 0.0f || u + v > 1.0f) return std::nullopt;

    float t = f * e2.dot(q);
    if (t < ray.tmin || t > ray.tmax) return std::nullopt;

    Vec3 normal = e1.cross(e2).normalized();
    if (normal.dot(ray.direction) > 0.0f) normal = -normal;
    return HitRecord{t, ray.at(t), normal, material_id};
}
