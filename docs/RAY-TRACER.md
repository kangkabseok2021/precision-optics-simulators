# Monte Carlo Optical Ray Tracer

A C++20 path tracer designed to simulate high-fidelity light transport and compute photometric luminance maps on automotive lighting components and refractive optics.

---

## Core Components

### 1. Spatial Partitioning: Axis-Aligned Bounding Box (AABB) & Bounding Volume Hierarchy (BVH)
Rendering high-density polygonal optics geometry (such as faceted headlamp reflectors or micro-lens arrays) via brute-force ray-intersection queries takes $O(N)$ operations per ray, which is computationally expensive.
- **BVH Structure**: We organize the scene triangles into a binary tree of Axis-Aligned Bounding Boxes.
- **Construction**: Splitting coordinates recursively along the longest bounding box axis using a median-split heuristic (`std::nth_element`). To prevent runtime heap fragmentation and maximize cache locality, we pre-allocate the entire tree node flat-array arena (`2N + 1` nodes) prior to recursion.
- **Traversal**: Rays traverse the bounding box tree hierarchy, testing intersection against bounding boxes first. If a ray misses a node's bounding box, the entire subtree is immediately pruned.
- **Complexity**: Accelerates ray-intersection queries from $O(N)$ down to $O(\log N)$ average-case complexity.

### 2. Möller-Trumbore Ray-Triangle Intersection
Tests ray intersection against individual geometric triangles.
- **Algorithm**: The Möller-Trumbore algorithm solves for barycentric coordinates ($u, v$) and the distance parameter ($t$) directly using vector cross and dot products.
- **Performance**: Eliminates the need to pre-compute or store plane equations for each triangle in memory, significantly reducing cache footprints.
- **Culling**: Implements double-sided geometric evaluation; if a ray hits a triangle from behind, the geometric normal is automatically flipped to align with the ray direction.

### 3. Physical Optics and BRDF Materials
Simulates the interaction of light rays with various optical boundaries:
- **Lambertian Diffuse**: Cosine-weighted hemisphere scattering.
- **Specular Mirror**: Ideal specular reflection matching the incident angle across the surface normal.
- **Dielectric (Glass/Plastic)**: Resolves refraction and reflection at boundaries using:
  - **Snell's Law**: Computes refraction angles based on relative refractive indices ($n_1 / n_2$).
  - **Total Internal Reflection (TIR)**: Automatically switches to 100% reflection when the refraction angle evaluates to an imaginary solution (light tries to exit a denser medium at a shallow angle).
  - **Schlick's Approximation**: Simulates the angle-dependent reflection probability (Fresnel equations) with a fast, computationally inexpensive power term.

### 4. Unbiased Monte Carlo Estimator and Russian Roulette
Path tracing works by tracing light paths backwards from the camera sensor into the scene.
- **Monte Carlo Integration**: Solves the recursive rendering equation by averaging randomly sampled paths.
- **Importance Sampling**: We sample scattering directions according to a cosine-weighted distribution on the hemisphere. This cancels out the geometric $\cos\theta$ term in the rendering equation, lowering numerical variance and leading to faster noise reduction.
- **Russian Roulette**: Tracing rays to a fixed depth limits maximum bounces and biases the result, while tracing rays infinitely leads to infinite runtime. We terminate paths dynamically after depth 3 with probability $1 - q$ (where throughput $q = \max(R, G, B)$). Remaining paths are scaled by $1/q$, resulting in an mathematically unbiased estimator that terminates in finite time.

### 5. Multithreaded Execution Flow
- **AppleClang Parallelization**: Standard C++ parallel algorithms (`std::execution::par_unseq`) are not supported natively in AppleClang.
- **Solution**: We divide the target image canvas into rows. Each row is rendered as an asynchronous task using `std::async` and `std::future`. Threads write results concurrently to a thread-safe `LuminanceMap` which uses lock-free array index writes or localized mutex strips to prevent data races.
