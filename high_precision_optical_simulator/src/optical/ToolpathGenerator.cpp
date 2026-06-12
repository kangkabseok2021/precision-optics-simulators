#include "ToolpathGenerator.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Standard natural-cubic-spline evaluation on interval [x_i, x_{i+1}].
// Let a = 1−t, b = t (t ∈ [0,1]):
//   S(t) = a·z[i] + b·z[i+1] + h²/6·((a³−a)·M[i] + (b³−b)·M[i+1])
// Second derivative at knots matches M[i] and M[i+1] by construction.
double ToolpathGenerator::SplineCoeffs::eval(size_t i, double t) const noexcept {
    double a   = 1.0 - t;
    double b   = t;
    double h26 = h_um * h_um / 6.0;
    return a * z[i] + b * z[i + 1]
         + h26 * ((a * a * a - a) * M[i] + (b * b * b - b) * M[i + 1]);
}

// Natural cubic spline: M_0 = M_{n} = 0.
// Interior second derivatives M_1 … M_{n-1} satisfy the tridiagonal system
//   M_{i-1} + 4·M_i + M_{i+1} = 6/h²·(z_{i+1} − 2·z_i + z_{i-1})
// Solved in O(n) via the Thomas (forward-elimination / back-substitution) algorithm.
ToolpathGenerator::SplineCoeffs
ToolpathGenerator::fitSpline(const std::vector<double>& z_um, double h_um) {
    SplineCoeffs sc;
    sc.z   = z_um;
    sc.h_um = h_um;
    sc.M.assign(z_um.size(), 0.0);   // natural boundary: M[0] = M[n] = 0

    const size_t n = z_um.size() - 1;
    if (n < 2) return sc;

    const size_t m = n - 1;   // interior unknowns M[1] … M[n-1]
    std::vector<double> rhs(m), c(m), d(m);

    const double inv_h2 = 6.0 / (h_um * h_um);
    for (size_t i = 0; i < m; ++i)
        rhs[i] = inv_h2 * (z_um[i + 2] - 2.0 * z_um[i + 1] + z_um[i]);

    // Forward sweep (uniform diagonal = 4, off-diagonal = 1)
    c[0] = 1.0 / 4.0;
    d[0] = rhs[0] / 4.0;
    for (size_t i = 1; i < m; ++i) {
        double denom = 4.0 - c[i - 1];
        c[i] = 1.0 / denom;
        d[i] = (rhs[i] - d[i - 1]) / denom;
    }

    // Back substitution — sc.M[1] … sc.M[n-1]
    sc.M[m] = d[m - 1];
    for (int i = static_cast<int>(m) - 2; i >= 0; --i)
        sc.M[i + 1] = d[i] - c[i] * sc.M[i + 2];

    return sc;
}

ToolpathGenerator::ToolpathGenerator(const LensSurface& surface, const GridParams& grid)
    : surface_(surface), grid_(grid)
{
    if (grid_.n_radial < 2)    throw std::invalid_argument("n_radial must be >= 2");
    if (grid_.n_angles < 1)    throw std::invalid_argument("n_angles must be >= 1");
    if (grid_.r_max_mm <= 0.0) throw std::invalid_argument("r_max_mm must be > 0");

    const size_t Nr = grid_.n_radial;
    const double h_mm = grid_.r_max_mm / static_cast<double>(Nr - 1);
    std::vector<double> z_raw(Nr);
    for (size_t i = 0; i < Nr; ++i)
        z_raw[i] = surface_.sag(i * h_mm) * 1000.0;   // mm → μm

    coeffs_ = fitSpline(z_raw, h_mm * 1000.0);        // h in μm for spline
}

std::vector<ToolpathPoint> ToolpathGenerator::generate() const {
    const size_t Nr    = grid_.n_radial;
    const size_t Na    = grid_.n_angles;
    const double h_mm  = grid_.r_max_mm / static_cast<double>(Nr - 1);
    const double dTheta = (Na > 1) ? (2.0 * M_PI / static_cast<double>(Na)) : 0.0;

    std::vector<ToolpathPoint> pts;
    pts.reserve(Na * Nr);

    for (size_t a = 0; a < Na; ++a) {
        double theta = a * dTheta;
        for (size_t i = 0; i < Nr; ++i) {
            pts.push_back({i * h_mm, theta, coeffs_.z[i]});
        }
    }
    return pts;
}

double ToolpathGenerator::evalAt(double r_mm) const {
    const size_t Nr   = grid_.n_radial;
    const double h_mm = grid_.r_max_mm / static_cast<double>(Nr - 1);

    r_mm = std::max(0.0, std::min(r_mm, grid_.r_max_mm));

    double  idx_f = r_mm / h_mm;
    size_t  i     = static_cast<size_t>(idx_f);
    if (i >= Nr - 1) i = Nr - 2;
    double t = idx_f - static_cast<double>(i);   // ∈ [0, 1)

    return coeffs_.eval(i, t);
}

DeviationStats ToolpathGenerator::analyseDeviation(
    const std::vector<ToolpathPoint>& reference,
    const std::vector<ToolpathPoint>& actual)
{
    if (reference.empty() || reference.size() != actual.size())
        return {0.0, 0.0, 0};

    double sum_sq = 0.0, peak = 0.0;
    for (size_t i = 0; i < reference.size(); ++i) {
        double d = std::abs(actual[i].z_um - reference[i].z_um);
        sum_sq += d * d;
        if (d > peak) peak = d;
    }
    return {
        std::sqrt(sum_sq / static_cast<double>(reference.size())),
        peak,
        reference.size()
    };
}
