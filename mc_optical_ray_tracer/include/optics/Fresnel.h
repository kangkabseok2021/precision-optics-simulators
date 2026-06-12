#pragma once

// Full dielectric Fresnel reflectance R = (Rs² + Rp²)/2
// Returns 1.0 on TIR
float fresnel_dielectric(float cos_i, float n1, float n2) noexcept;

// Schlick approximation: R0 + (1-R0)(1-cosθ)⁵
float fresnel_schlick(float cos_theta, float n1, float n2) noexcept;

// Snell's law: fills cos_t, returns false on TIR
bool snell_refract(float cos_i, float n1, float n2, float& cos_t) noexcept;
