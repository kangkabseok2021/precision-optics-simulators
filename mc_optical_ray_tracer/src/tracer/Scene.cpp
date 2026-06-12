#include "tracer/Scene.h"

void Scene::add_triangle(Triangle t, Material m) {
    t.material_id = static_cast<int>(materials_.size());
    materials_.push_back(m);
    tris_.push_back(t);
}

void Scene::commit() {
    bvh_.build(tris_);
}

std::optional<HitRecord> Scene::intersect(const Ray& ray) const noexcept {
    return bvh_.intersect(ray);
}
