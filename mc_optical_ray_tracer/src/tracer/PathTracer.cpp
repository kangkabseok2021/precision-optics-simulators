#include "tracer/PathTracer.h"
#include "optics/BRDF.h"
#include "optics/Fresnel.h"
#include <future>
#include <thread>
#include <vector>
#include <numeric>
#include <random>

LuminanceMap PathTracer::render(const Scene& scene, const Camera& camera,
                                int width, int height, int spp) const {
    LuminanceMap lm{width, height};

    int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0) num_threads = 4;
    std::vector<std::future<void>> futures;

    int rows_per_thread = (height + num_threads - 1) / num_threads;

    for (int t = 0; t < num_threads; ++t) {
        int start_row = t * rows_per_thread;
        int end_row = std::min(start_row + rows_per_thread, height);
        if (start_row >= end_row) continue;

        futures.push_back(std::async(std::launch::async, [this, &scene, &camera, width, height, start_row, end_row, spp, &lm]() {
            // Seed thread-local RNG with unique value per thread to prevent identical samples
            static thread_local std::mt19937 thread_rng{
                std::random_device{}() ^ 
                static_cast<unsigned int>(std::hash<std::thread::id>{}(std::this_thread::get_id()))
            };
            std::uniform_real_distribution<float> thread_dist(0.0f, 1.0f);

            for (int row = start_row; row < end_row; ++row) {
                for (int col = 0; col < width; ++col) {
                    for (int s = 0; s < spp; ++s) {
                        float u = (col + thread_dist(thread_rng)) / width;
                        float v = (row + thread_dist(thread_rng)) / height;
                        Ray ray = camera.generate_ray(u, v);
                        Vec3 L = trace(scene, ray, 0);
                        lm.add(col, row, L);
                    }
                }
            }
        }));
    }

    for (auto& f : futures) {
        f.get();
    }

    return lm;
}

Vec3 PathTracer::trace(const Scene& scene, const Ray& ray, int depth) const {
    if (depth >= max_depth_) return Vec3{};

    auto hit = scene.intersect(ray);
    if (!hit) return Vec3{0.2f, 0.2f, 0.4f};  // sky colour

    const Material& mat = scene.get_material(hit->material_id);

    // Russian roulette
    Vec3  throughput = mat.albedo;
    float q = std::max({throughput.x, throughput.y, throughput.z});
    if (depth > 3) {
        static thread_local std::mt19937 thread_rng{std::random_device{}()};
        static thread_local std::uniform_real_distribution<float> thread_dist(0.0f, 1.0f);
        if (thread_dist(thread_rng) > q || q < 1e-6f) return Vec3{};
        throughput = throughput * (1.0f / q);
    }

    if (mat.type == MaterialType::Mirror) {
        Vec3 reflected = reflect(ray.direction, hit->normal);
        Ray  refl_ray{hit->point, reflected};
        return throughput * trace(scene, refl_ray, depth + 1);
    }

    // Lambertian diffuse
    static thread_local std::mt19937 thread_rng{std::random_device{}()};
    static thread_local std::uniform_real_distribution<float> thread_dist(0.0f, 1.0f);
    float xi1 = thread_dist(thread_rng), xi2 = thread_dist(thread_rng);
    // Build local frame with z = hit normal
    Vec3 n = hit->normal;
    Vec3 perp = std::abs(n.x) > 0.9f ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 t_ax = n.cross(perp).normalized();
    Vec3 b_ax = n.cross(t_ax);

    float pdf;
    Vec3 local_wi = cosine_sample_hemisphere(xi1, xi2, pdf);
    Vec3 wi = t_ax * local_wi.x + b_ax * local_wi.y + n * local_wi.z;

    Ray scatter{hit->point, wi};
    Vec3 brdf   = lambertian_f(mat);
    float cos_t = local_wi.z;
    if (pdf > 0.0f) {
        return (brdf * cos_t / pdf) * trace(scene, scatter, depth + 1);
    }
    return Vec3{};
}
