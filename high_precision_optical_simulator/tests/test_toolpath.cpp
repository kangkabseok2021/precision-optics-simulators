#include <gtest/gtest.h>
#include "optical/LensSurface.h"
#include "optical/ToolpathGenerator.h"
#include <cmath>
#include <vector>

// ── LensSurface ──────────────────────────────────────────────────────────────

TEST(LensSurfaceTest, ZeroSagAtCenter) {
    LensSurface surf{100.0, 0.0, {}};
    EXPECT_DOUBLE_EQ(surf.sag(0.0), 0.0);
}

TEST(LensSurfaceTest, SphericalApproximation) {
    // For r << R: sag(r) ≈ r²/(2R)
    LensSurface surf{100.0, 0.0, {}};
    double r = 1.0;   // mm, r/R = 0.01 (well within paraxial region)
    double approx = r * r / (2.0 * surf.radius);   // mm
    EXPECT_NEAR(surf.sag(r), approx, 1e-6);         // < 1 nm error
}

TEST(LensSurfaceTest, ZernukeCoefficientAffectsSag) {
    LensSurface base{100.0, 0.0, {}};
    LensSurface corr{100.0, 0.0, {}};
    corr.zernike[0] = 0.001;   // mm
    // At r = 50 mm, the first Zernike term = 0.001 * (50/100)² = 0.00025 mm = 0.25 μm
    EXPECT_NE(base.sag(50.0), corr.sag(50.0));
    EXPECT_NEAR(corr.sag(50.0) - base.sag(50.0), 0.001 * 0.25, 1e-9);
}

TEST(LensSurfaceTest, OutsideApertureReturnsZero) {
    // Oblate (K=1): disc = 1 − 2r²/R² → goes negative when r > R/√2.
    // R=10mm: aperture edge at r = 10/√2 ≈ 7.07mm.  At r=9mm disc < 0 → sag=0.
    LensSurface surf{10.0, 1.0, {}};
    EXPECT_DOUBLE_EQ(surf.sag(9.0), 0.0);
}

// ── ToolpathGenerator ────────────────────────────────────────────────────────

TEST(ToolpathTest, PointCountMatchesGrid) {
    LensSurface surf{100.0, 0.0, {}};
    GridParams  grid{10.0, 50, 12};
    ToolpathGenerator gen(surf, grid);
    auto pts = gen.generate();
    EXPECT_EQ(pts.size(), 50u * 12u);
}

TEST(ToolpathTest, RMSDeviationZeroWhenIdentical) {
    LensSurface surf{100.0, 0.0, {}};
    GridParams  grid{10.0, 20, 4};
    ToolpathGenerator gen(surf, grid);
    auto ref = gen.generate();
    auto stats = ToolpathGenerator::analyseDeviation(ref, ref);
    EXPECT_DOUBLE_EQ(stats.rms_um,  0.0);
    EXPECT_DOUBLE_EQ(stats.peak_um, 0.0);
}

TEST(ToolpathTest, RMSDeviationForUniformOffset) {
    LensSurface surf{100.0, 0.0, {}};
    GridParams  grid{10.0, 20, 4};
    ToolpathGenerator gen(surf, grid);
    auto ref    = gen.generate();
    auto actual = ref;
    constexpr double OFFSET = 0.5;   // μm
    for (auto& p : actual) p.z_um += OFFSET;
    auto stats = ToolpathGenerator::analyseDeviation(ref, actual);
    EXPECT_NEAR(stats.rms_um,  OFFSET, 1e-10);
    EXPECT_NEAR(stats.peak_um, OFFSET, 1e-10);
    EXPECT_EQ(stats.n, ref.size());
}

TEST(ToolpathTest, SplineEvalAtKnotIsExact) {
    // The spline passes exactly through all knot values by construction.
    LensSurface surf{100.0, 0.0, {}};
    GridParams  grid{10.0, 101, 1};   // h = 0.1 mm per knot
    ToolpathGenerator gen(surf, grid);

    // Check every 10th knot position
    double h = 10.0 / 100.0;
    for (int k = 0; k <= 100; k += 10) {
        double r       = k * h;
        double expected = surf.sag(r) * 1000.0;   // μm
        EXPECT_NEAR(gen.evalAt(r), expected, 1e-9)
            << "Spline deviation at knot r=" << r << " mm";
    }
}
