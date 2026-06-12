#pragma once
#include "geometry/Vec3.h"

enum class MaterialType { Lambertian, Mirror, Dielectric };

struct Material {
    MaterialType type{MaterialType::Lambertian};
    Vec3  albedo{0.8f, 0.8f, 0.8f};
    float ior{1.5f};
    float roughness{0.0f};
};
