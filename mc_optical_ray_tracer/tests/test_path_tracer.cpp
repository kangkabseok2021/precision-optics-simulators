#include <gtest/gtest.h>
#include "tracer/Camera.h"
#include "tracer/Scene.h"
#include "tracer/PathTracer.h"
#include "lighting/LuminanceMap.h"
#include "optics/BRDF.h"
#include <cmath>
#include <numeric>

// Center pixel ray should point approximately along -z for default camera
TEST(Camera, CenterRay) {
    Camera cam{Vec3{0,0,1}, Vec3{0,0,0}, Vec3{0,1,0}, 90.0f, 1.0f};
    Ray r = cam.generate_ray(0.5f, 0.5f);
    EXPECT_NEAR(r.direction.x, 0.0f, 1e-4f);
    EXPECT_NEAR(r.direction.y, 0.0f, 1e-4f);
    EXPECT_LT(r.direction.z, 0.0f);  // pointing into scene
}

TEST(LuminanceMap, MeanAccumulation) {
    LuminanceMap lm{2, 2};
    lm.add(0, 0, Vec3{1.0f, 0.5f, 0.25f});
    lm.add(0, 0, Vec3{1.0f, 0.5f, 0.25f});
    Vec3 mean = lm.get_mean(0, 0);
    EXPECT_NEAR(mean.x, 1.0f,  1e-5f);
    EXPECT_NEAR(mean.y, 0.5f,  1e-5f);
    EXPECT_NEAR(mean.z, 0.25f, 1e-5f);
}

// Cosine hemisphere mean(cosθ) ≈ 2/3 over N=5000 samples
TEST(BRDF, CosineHemisphereMean) {
    float sum = 0.0f;
    int N = 5000;
    for (int i = 0; i < N; ++i) {
        float xi1 = (i + 0.5f) / N;
        float xi2 = (i * 0.618033f - std::floor(i * 0.618033f));
        float pdf;
        Vec3 d = cosine_sample_hemisphere(xi1, xi2, pdf);
        sum += d.z;
    }
    EXPECT_NEAR(sum / N, 2.0f/3.0f, 0.01f);
}

#include "lighting/Photometry.h"

// Point source directly above at 1 m, intensity 1 cd, normal pointing up → E = 1 lux
TEST(Photometry, InverseSquareLaw) {
    // source at (0,1,0), point at (0,0,0), normal (0,1,0), I=1 cd
    float E = compute_illuminance(Vec3{0,1,0}, Vec3{0,0,0}, Vec3{0,1,0}, 1.0f);
    EXPECT_NEAR(E, 1.0f, 1e-4f);
}

// Source at (1,1,0), point at (0,0,0), normal (0,1,0) — 45° incidence, r=√2
// E = I*cosθ/r² = 1*(1/√2)/(2) = 1/(2√2) ≈ 0.3536
TEST(Photometry, CosineFalloff) {
    Vec3 source{1.0f,1.0f,0.0f}, point{0,0,0}, normal{0,1,0};
    float E = compute_illuminance(source, point, normal, 1.0f);
    EXPECT_NEAR(E, 1.0f / (2.0f * std::sqrt(2.0f)), 1e-4f);
}
