#pragma once
#include "geometry/Vec3.h"

float compute_illuminance(Vec3 source, Vec3 point, Vec3 normal, float intensity) noexcept;
