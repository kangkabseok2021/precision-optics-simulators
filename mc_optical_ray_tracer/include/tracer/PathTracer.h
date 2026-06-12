#pragma once
#include "tracer/Scene.h"
#include "tracer/Camera.h"
#include "lighting/LuminanceMap.h"

class PathTracer {
public:
    explicit PathTracer(int max_depth = 8) : max_depth_(max_depth) {}

    LuminanceMap render(const Scene& scene, const Camera& camera,
                        int width, int height, int spp) const;

private:
    Vec3 trace(const Scene& scene, const Ray& ray, int depth) const;
    int max_depth_;
};
