#pragma once
#include "geometry/Triangle.h"
#include "geometry/BVH.h"
#include "optics/Material.h"
#include <vector>
#include <optional>

class Scene {
public:
    void add_triangle(Triangle t, Material m);
    void commit();  // builds BVH
    std::optional<HitRecord> intersect(const Ray& ray) const noexcept;
    const Material& get_material(int id) const noexcept { return materials_[id]; }

private:
    std::vector<Triangle> tris_;
    std::vector<Material> materials_;
    BVH bvh_;
};
