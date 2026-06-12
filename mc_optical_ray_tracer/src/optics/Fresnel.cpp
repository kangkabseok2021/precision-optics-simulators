#include "optics/Fresnel.h"
#include <cmath>

float fresnel_dielectric(float cos_i, float n1, float n2) noexcept {
    float sin_t_sq = (n1/n2)*(n1/n2)*(1.0f - cos_i*cos_i);
    if (sin_t_sq >= 1.0f) return 1.0f;  // TIR
    float cos_t = std::sqrt(1.0f - sin_t_sq);
    float rs = (n1*cos_i - n2*cos_t) / (n1*cos_i + n2*cos_t);
    float rp = (n2*cos_i - n1*cos_t) / (n2*cos_i + n1*cos_t);
    return (rs*rs + rp*rp) * 0.5f;
}

float fresnel_schlick(float cos_theta, float n1, float n2) noexcept {
    float r0 = (n1 - n2) / (n1 + n2);
    r0 *= r0;
    float x = 1.0f - cos_theta;
    return r0 + (1.0f - r0) * x*x*x*x*x;
}

bool snell_refract(float cos_i, float n1, float n2, float& cos_t) noexcept {
    float sin_t_sq = (n1/n2)*(n1/n2)*(1.0f - cos_i*cos_i);
    if (sin_t_sq >= 1.0f) return false;
    cos_t = std::sqrt(1.0f - sin_t_sq);
    return true;
}
