#pragma once
#include "geometry/Vec3.h"
#include "optics/Material.h"

constexpr float kPi = 3.14159265358979323846f;

Vec3 cosine_sample_hemisphere(float xi1, float xi2, float& pdf) noexcept;
Vec3 lambertian_f(const Material& mat) noexcept;
Vec3 reflect(const Vec3& wi, const Vec3& normal) noexcept;
