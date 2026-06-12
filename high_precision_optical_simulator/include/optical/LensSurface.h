#pragma once
#include <array>
#include <cmath>

// Aspheric lens surface with optional Zernike radial corrections.
//
// z(r) = r² / (R·(1 + √(1 − (1+K)·r²/R²)))  +  Σᵢ aᵢ·(r/R)^(2i+2)
//
// Simplified radial Zernike expansion — each coefficient aᵢ (mm) scales a
// higher-order even-power term.  Sufficient for rotationally-symmetric
// prescription lenses; toric and free-form surfaces require 2D Zernike maps.
struct LensSurface {
    double radius;                    // base radius of curvature R (mm), > 0
    double conic;                     // conic constant K (0=sphere, −1=paraboloid)
    std::array<double, 6> zernike{};  // radial correction coefficients aᵢ (mm)

    // Sag height at radial distance r from the optical axis (mm).
    // Returns 0.0 for r outside the valid aperture (discriminant < 0).
    double sag(double r) const noexcept {
        double r2   = r * r;
        double disc = 1.0 - (1.0 + conic) * r2 / (radius * radius);
        if (disc < 0.0) return 0.0;
        double base = r2 / (radius * (1.0 + std::sqrt(disc)));
        double rn   = r / radius;
        double correction = 0.0;
        double rn_pow = rn * rn;          // rn^2
        for (double a : zernike) {
            correction += a * rn_pow;
            rn_pow     *= rn * rn;        // rn^(2i+2) for next i
        }
        return base + correction;
    }
};
