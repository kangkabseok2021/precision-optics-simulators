#pragma once
#include "Vec3.h"
#include "Ray.h"
#include <algorithm>

struct AABB {
    Vec3 min_pt, max_pt;

    bool intersect(const Ray& ray) const noexcept;

    static AABB surrounding(const AABB& a, const AABB& b) noexcept {
        return {
            Vec3{std::min(a.min_pt.x, b.min_pt.x),
                 std::min(a.min_pt.y, b.min_pt.y),
                 std::min(a.min_pt.z, b.min_pt.z)},
            Vec3{std::max(a.max_pt.x, b.max_pt.x),
                 std::max(a.max_pt.y, b.max_pt.y),
                 std::max(a.max_pt.z, b.max_pt.z)}
        };
    }
};
