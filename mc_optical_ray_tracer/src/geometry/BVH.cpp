#include "geometry/BVH.h"
#include <algorithm>
#include <stack>
#include <numeric>

static AABB tri_bounds(const Triangle& t) {
    return {
        Vec3{std::min({t.v0.x, t.v1.x, t.v2.x}),
             std::min({t.v0.y, t.v1.y, t.v2.y}),
             std::min({t.v0.z, t.v1.z, t.v2.z})},
        Vec3{std::max({t.v0.x, t.v1.x, t.v2.x}),
             std::max({t.v0.y, t.v1.y, t.v2.y}),
             std::max({t.v0.z, t.v1.z, t.v2.z})}
    };
}

static Vec3 tri_centroid(const Triangle& t) {
    return (t.v0 + t.v1 + t.v2) * (1.0f/3.0f);
}

void BVH::build(std::vector<Triangle>& tris) {
    tris_ = tris;
    int n = static_cast<int>(tris_.size());
    nodes_.clear();
    if (n == 0) return;
    nodes_.reserve(2 * n + 1);  // pre-reserve to prevent reallocation during recursion
    build_recursive(0, n);
}

int BVH::build_recursive(int start, int end) {
    int idx = static_cast<int>(nodes_.size());
    nodes_.push_back({});
    
    AABB bounds = compute_bounds(start, end);
    nodes_[idx].bounds = bounds;

    if (end - start <= 2) {
        nodes_[idx].tri_start = start;
        nodes_[idx].tri_count = end - start;
        return idx;
    }

    // Find longest axis
    Vec3 extent = bounds.max_pt - bounds.min_pt;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;

    // Median split
    int mid = (start + end) / 2;
    std::nth_element(tris_.begin() + start, tris_.begin() + mid, tris_.begin() + end,
        [axis](const Triangle& a, const Triangle& b) {
            return tri_centroid(a)[axis] < tri_centroid(b)[axis];
        });

    // Build children
    int left  = build_recursive(start, mid);
    int right = build_recursive(mid,   end);
    nodes_[idx].left  = left;
    nodes_[idx].right = right;
    return idx;
}

AABB BVH::compute_bounds(int start, int end) const {
    AABB b = tri_bounds(tris_[start]);
    for (int i = start + 1; i < end; ++i)
        b = AABB::surrounding(b, tri_bounds(tris_[i]));
    return b;
}

std::optional<HitRecord> BVH::intersect(const Ray& ray) const noexcept {
    if (nodes_.empty()) return std::nullopt;
    return intersect_node(0, ray);
}

std::optional<HitRecord> BVH::intersect_node(int idx, const Ray& ray) const noexcept {
    const BVHNode& node = nodes_[idx];
    if (!node.bounds.intersect(ray)) return std::nullopt;

    if (node.is_leaf()) {
        std::optional<HitRecord> best;
        Ray r = ray;
        for (int i = node.tri_start; i < node.tri_start + node.tri_count; ++i) {
            auto h = tris_[i].intersect(r);
            if (h) { best = h; r.tmax = h->t; }
        }
        return best;
    }

    auto lh = intersect_node(node.left,  ray);
    Ray  r2 = ray;
    if (lh) r2.tmax = lh->t;
    auto rh = intersect_node(node.right, r2);
    return rh ? rh : lh;
}
