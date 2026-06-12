#pragma once
#include "LensSurface.h"
#include <vector>

struct ToolpathPoint {
    double r_mm;       // radial distance from optical axis (mm)
    double theta_rad;  // azimuthal angle
    double z_um;       // sag height in micrometres
};

struct DeviationStats {
    double rms_um;   // RMS surface error
    double peak_um;  // maximum absolute deviation
    size_t n;        // points analysed
};

struct GridParams {
    double r_max_mm;  // aperture radius (mm)
    size_t n_radial;  // radial sample count per profile
    size_t n_angles;  // number of angular profiles
};

class ToolpathGenerator {
public:
    ToolpathGenerator(const LensSurface& surface, const GridParams& grid);

    // Generate toolpath: n_angles × n_radial ToolpathPoints ordered by (angle, radius).
    // Each radial profile is cubic-spline-fitted via the Thomas algorithm.
    std::vector<ToolpathPoint> generate() const;

    // Evaluate the fitted spline at an arbitrary radial position.
    // Returns sag in μm.  Complexity: O(1).
    double evalAt(double r_mm) const;

    // Compare reference (generate()) vs measured actual positions.
    // actual must have the same length and ordering as generate() output.
    static DeviationStats analyseDeviation(
        const std::vector<ToolpathPoint>& reference,
        const std::vector<ToolpathPoint>& actual);

private:
    struct SplineCoeffs {
        std::vector<double> z;    // knot values (μm), length n
        std::vector<double> M;    // second derivatives at knots, length n
        double h_um;              // uniform knot spacing in μm

        // Evaluate S at fractional index i + t, t ∈ [0, 1).
        // Uses the standard cubic-spline formula: a·z[i] + b·z[i+1] + h²/6·(…)
        double eval(size_t i, double t) const noexcept;
    };

    // Natural cubic spline via Thomas algorithm (tridiagonal solve O(N)).
    static SplineCoeffs fitSpline(const std::vector<double>& z_um, double h_um);

    LensSurface  surface_;
    GridParams   grid_;
    SplineCoeffs coeffs_;   // pre-computed in constructor
};
