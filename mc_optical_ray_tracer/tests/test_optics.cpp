#include <gtest/gtest.h>
#include "optics/Fresnel.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// n=1→1.5: R at normal incidence = ((1-1.5)/(1+1.5))² = 0.04
TEST(Snell, NormalIncidence) {
    float cos_t;
    bool refracted = snell_refract(1.0f, 1.0f, 1.5f, cos_t);
    EXPECT_TRUE(refracted);
    EXPECT_NEAR(cos_t, 1.0f, 1e-5f);
}

// air→glass 30° incidence: sinθt = (1.0/1.5)*sin30 = 1/3; cosθt = sqrt(1-1/9) ≈ 0.9428
TEST(Snell, GlassRefraction) {
    float cos_i = std::cos(30.0f * M_PI / 180.0f);  // ≈ 0.8660
    float cos_t;
    bool refracted = snell_refract(cos_i, 1.0f, 1.5f, cos_t);
    EXPECT_TRUE(refracted);
    EXPECT_NEAR(cos_t, std::sqrt(1.0f - (1.0f/9.0f)), 1e-4f);
}

// glass→air at 45°: sinθt = 1.5/1.0 * sin45 = 1.5*0.707 ≈ 1.06 > 1 → TIR
TEST(Snell, TotalInternalReflection) {
    float cos_i = std::cos(45.0f * M_PI / 180.0f);
    float cos_t;
    bool refracted = snell_refract(cos_i, 1.5f, 1.0f, cos_t);
    EXPECT_FALSE(refracted);
}

// Fresnel at normal incidence n1=1, n2=1.5: R = 0.04
TEST(Fresnel, NormalIncidenceDielectric) {
    float R = fresnel_dielectric(1.0f, 1.0f, 1.5f);
    EXPECT_NEAR(R, 0.04f, 1e-4f);
}

#include "optics/BRDF.h"
#include "optics/Material.h"

// TIR test: glass->air above critical angle, fresnel_dielectric returns 1.0
TEST(Fresnel, TIRDielectric) {
    float cos_i = std::cos(45.0f * M_PI / 180.0f);
    float R = fresnel_dielectric(cos_i, 1.5f, 1.0f);
    EXPECT_NEAR(R, 1.0f, 1e-5f);
}

// Schlick matches Fresnel exactly at normal incidence
TEST(Fresnel, SchlickNormalIncidence) {
    float R_exact   = fresnel_dielectric(1.0f, 1.0f, 1.5f);  // 0.04
    float R_schlick = fresnel_schlick(1.0f, 1.0f, 1.5f);
    EXPECT_NEAR(R_schlick, R_exact, 1e-5f);
}

// Lambertian BRDF energy conservation: mean(f*cosθ/pdf) = albedo for N=1000 samples
TEST(BRDF, LambertianEnergyConservation) {
    Material mat{MaterialType::Lambertian, Vec3{1.0f,1.0f,1.0f}, 1.0f, 0.0f};
    float sum = 0.0f;
    int N = 1000;
    for (int i = 0; i < N; ++i) {
        float xi1 = (i + 0.5f) / N;
        float xi2 = (i * 0.618033f - std::floor(i * 0.618033f));
        float pdf;
        Vec3 wi = cosine_sample_hemisphere(xi1, xi2, pdf);
        float cos_theta = wi.z;
        Vec3 f = lambertian_f(mat);
        if (pdf > 0.0f) {
            sum += f.dot(Vec3{1,1,1}) / 3.0f * cos_theta / pdf;
        }
    }
    EXPECT_NEAR(sum / N, 1.0f, 0.02f);
}
