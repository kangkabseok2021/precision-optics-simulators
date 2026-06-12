#include "tracer/Scene.h"
#include "tracer/Camera.h"
#include "tracer/PathTracer.h"
#include <fstream>
#include <algorithm>
#include <cstdio>

int main() {
    constexpr int W = 256, H = 256, SPP = 16;

    Scene scene;
    Material white{MaterialType::Lambertian, Vec3{0.8f,0.8f,0.8f}, 1.0f, 0.0f};
    Material red  {MaterialType::Lambertian, Vec3{0.8f,0.1f,0.1f}, 1.0f, 0.0f};
    Material green{MaterialType::Lambertian, Vec3{0.1f,0.8f,0.1f}, 1.0f, 0.0f};
    Material light{MaterialType::Lambertian, Vec3{5.0f,5.0f,5.0f}, 1.0f, 0.0f};

    // Floor
    scene.add_triangle(Triangle{Vec3{-1,-1,-1},Vec3{1,-1,-1},Vec3{1,-1,1},0}, white);
    scene.add_triangle(Triangle{Vec3{-1,-1,-1},Vec3{1,-1,1},Vec3{-1,-1,1},0}, white);
    // Back wall
    scene.add_triangle(Triangle{Vec3{-1,-1,-2},Vec3{1,-1,-2},Vec3{1,1,-2},0}, white);
    scene.add_triangle(Triangle{Vec3{-1,-1,-2},Vec3{1,1,-2},Vec3{-1,1,-2},0}, white);
    // Left wall (red)
    scene.add_triangle(Triangle{Vec3{-1,-1,-2},Vec3{-1,-1,1},Vec3{-1,1,1},0}, red);
    scene.add_triangle(Triangle{Vec3{-1,-1,-2},Vec3{-1,1,1},Vec3{-1,1,-2},0}, red);
    // Right wall (green)
    scene.add_triangle(Triangle{Vec3{1,-1,-2},Vec3{1,1,-2},Vec3{1,1,1},0}, green);
    scene.add_triangle(Triangle{Vec3{1,-1,-2},Vec3{1,1,1},Vec3{1,-1,1},0}, green);
    // Ceiling
    scene.add_triangle(Triangle{Vec3{-1,1,-2},Vec3{1,1,-2},Vec3{1,1,1},0}, white);
    scene.add_triangle(Triangle{Vec3{-1,1,-2},Vec3{1,1,1},Vec3{-1,1,1},0}, white);
    // Light patch on ceiling
    scene.add_triangle(Triangle{Vec3{-0.3f,0.99f,-1.3f},Vec3{0.3f,0.99f,-1.3f},Vec3{0.3f,0.99f,-0.7f},0}, light);
    scene.add_triangle(Triangle{Vec3{-0.3f,0.99f,-1.3f},Vec3{0.3f,0.99f,-0.7f},Vec3{-0.3f,0.99f,-0.7f},0}, light);
    scene.commit();

    Camera cam{Vec3{0,0,2}, Vec3{0,0,-1}, Vec3{0,1,0}, 60.0f, 1.0f};
    PathTracer pt{8};
    LuminanceMap lm = pt.render(scene, cam, W, H, SPP);

    std::ofstream ppm("render.ppm");
    ppm << "P3\n" << W << " " << H << "\n255\n";
    for (int row = H-1; row >= 0; --row) {
        for (int col = 0; col < W; ++col) {
            Vec3 c = lm.get_mean(col, row);
            // gamma 2 tone-map
            auto clamp = [](float v){ return std::min(1.0f, std::max(0.0f, v)); };
            int r = static_cast<int>(255.99f * std::sqrt(clamp(c.x)));
            int g = static_cast<int>(255.99f * std::sqrt(clamp(c.y)));
            int b = static_cast<int>(255.99f * std::sqrt(clamp(c.z)));
            ppm << r << " " << g << " " << b << "\n";
        }
    }
    std::printf("Wrote render.ppm (%dx%d, %d spp)\n", W, H, SPP);
    return 0;
}
