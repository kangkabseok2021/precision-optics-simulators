#include <gtest/gtest.h>
#include "geometry/Triangle.h"

// Triangle v0=(0,0,0) v1=(1,0,0) v2=(0,1,0), ray from (0.25,0.25,1) dir (0,0,-1)
TEST(RayTriangle, Hit) {
    Triangle tri{Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}, 0};
    Ray ray{Vec3{0.25f,0.25f,1.f}, Vec3{0,0,-1.f}};
    auto hit = tri.intersect(ray);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->t, 1.0f, 1e-5f);
    EXPECT_NEAR(hit->point.x, 0.25f, 1e-5f);
    EXPECT_NEAR(hit->point.y, 0.25f, 1e-5f);
    EXPECT_NEAR(hit->point.z, 0.0f,  1e-5f);
}

TEST(RayTriangle, MissParallelRay) {
    Triangle tri{Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}, 0};
    Ray ray{Vec3{0,0,1}, Vec3{1,0,0}};  // parallel to plane
    EXPECT_FALSE(tri.intersect(ray).has_value());
}

TEST(RayTriangle, MissBehind) {
    Triangle tri{Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}, 0};
    // origin below plane, pointing away
    Ray ray{Vec3{0.25f,0.25f,-1.f}, Vec3{0,0,-1.f}};
    EXPECT_FALSE(tri.intersect(ray).has_value());
}

TEST(RayTriangle, MissOutside) {
    Triangle tri{Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}, 0};
    // u+v = 1.5 > 1
    Ray ray{Vec3{0.75f,0.75f,1.f}, Vec3{0,0,-1.f}};
    EXPECT_FALSE(tri.intersect(ray).has_value());
}

#include "geometry/AABB.h"
#include "geometry/BVH.h"

TEST(AABB, RayHit) {
    AABB box{Vec3{0,0,0}, Vec3{1,1,1}};
    // ray from (-1, 0.5, 0.5) pointing +x
    Ray ray{Vec3{-1.f,0.5f,0.5f}, Vec3{1,0,0}};
    EXPECT_TRUE(box.intersect(ray));
}

TEST(AABB, RayMiss) {
    AABB box{Vec3{0,0,0}, Vec3{1,1,1}};
    // ray from (-1, 2, 0.5) pointing +x — passes above box
    Ray ray{Vec3{-1.f,2.f,0.5f}, Vec3{1,0,0}};
    EXPECT_FALSE(box.intersect(ray));
}

TEST(BVH, SingleTriangleHit) {
    std::vector<Triangle> tris = {
        Triangle{Vec3{0,0,0}, Vec3{1,0,0}, Vec3{0,1,0}, 0}
    };
    BVH bvh;
    bvh.build(tris);
    Ray ray{Vec3{0.25f,0.25f,1.f}, Vec3{0,0,-1.f}};
    auto hit = bvh.intersect(ray);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->t, 1.0f, 1e-5f);
}

TEST(BVH, ReturnsNearestHit) {
    // Two triangles: one at z=0 and one at z=0.5
    std::vector<Triangle> tris = {
        Triangle{Vec3{0,0,0},   Vec3{1,0,0},   Vec3{0,1,0},   0},  // z=0, t=1.0
        Triangle{Vec3{0,0,0.5f},Vec3{1,0,0.5f},Vec3{0,1,0.5f},1}   // z=0.5, t=0.5
    };
    BVH bvh;
    bvh.build(tris);
    Ray ray{Vec3{0.25f,0.25f,1.f}, Vec3{0,0,-1.f}};
    auto hit = bvh.intersect(ray);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->t, 0.5f, 1e-5f);  // z=0.5 triangle is closer
}
