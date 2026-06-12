#pragma once
#include "geometry/Triangle.h"
#include "geometry/AABB.h"
#include <vector>
#include <optional>

struct BVHNode {
    AABB  bounds;
    int   left{-1}, right{-1};
    int   tri_start{-1}, tri_count{0};
    bool  is_leaf() const noexcept { return tri_count > 0; }
};

class BVH {
public:
    void build(std::vector<Triangle>& tris);
    std::optional<HitRecord> intersect(const Ray& ray) const noexcept;

private:
    std::vector<BVHNode>  nodes_;
    std::vector<Triangle> tris_;

    int  build_recursive(int start, int end);
    AABB compute_bounds(int start, int end) const;
    std::optional<HitRecord> intersect_node(int node_idx, const Ray& ray) const noexcept;
};
