#pragma once
#include "Vec3.h"
#include "Ray.h"
#include <optional>

struct HitRecord {
    float t;
    Vec3  point;
    Vec3  normal;
    int   material_id;
};

struct Triangle {
    Vec3 v0, v1, v2;
    int  material_id{0};

    std::optional<HitRecord> intersect(const Ray& ray) const noexcept;
};
