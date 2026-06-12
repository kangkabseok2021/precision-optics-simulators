# Optics Mathematics Reference

## Möller-Trumbore Ray-Triangle Intersection

Given ray **o** + t**d** and triangle (v₀, v₁, v₂):

```
e₁ = v₁ - v₀,  e₂ = v₂ - v₀
h  = d × e₂
a  = e₁ · h            (determinant)
f  = 1/a
u  = f · (o - v₀) · h  ∈ [0,1]
q  = (o - v₀) × e₁
v  = f · d · q          ∈ [0,1], u+v ≤ 1
t  = f · e₂ · q         ∈ [tmin, tmax]
```

Complexity: O(1) per ray-triangle pair. Back-face culling: flip normal if `d·n > 0`.

## AABB Slab Intersection

For each axis i: compute t_near = (min[i] - o[i]) / d[i], t_far = (max[i] - o[i]) / d[i].
Swap if d[i] < 0. Intersect → max(t_near) < min(t_far).

## BVH Median-Split

Build: O(n log n) — `std::nth_element` along longest axis at each node.
Traversal: O(log n) average. Pre-reserve `2n+1` nodes before recursion to prevent reallocation.

## Snell's Law

```
n₁ sin θᵢ = n₂ sin θₜ
sin²θₜ = (n₁/n₂)² (1 - cos²θᵢ)
```

Total internal reflection (TIR): sin²θₜ ≥ 1 when n₁/n₂ · sin θᵢ > 1.

## Fresnel Equations (Dielectric)

```
Rₛ = (n₁ cosθᵢ - n₂ cosθₜ) / (n₁ cosθᵢ + n₂ cosθₜ)
Rₚ = (n₂ cosθᵢ - n₁ cosθₜ) / (n₂ cosθᵢ + n₁ cosθₜ)
R  = (Rₛ² + Rₚ²) / 2
```

At normal incidence (n₁=1, n₂=1.5): R = ((1-1.5)/(1+1.5))² = 0.04.

## Schlick Approximation

```
R₀ = ((n₁-n₂)/(n₁+n₂))²
R(θ) = R₀ + (1-R₀)(1-cosθ)⁵
```

Error vs. full Fresnel < 0.3% for glass (n=1.5) across all angles.

## Monte Carlo Path Integral Estimator

```
L ≈ (1/N) Σᵢ f(ωᵢ) · Lᵢ · cosθᵢ / p(ωᵢ)
```

Cosine-weighted hemisphere sampling eliminates the cosθ/p(ω) factor:
- p(ω) = cosθ/π  →  f·cosθ/p = f·π
- E[cosθ] = 2/3 (analytically)

## Russian Roulette Termination

Throughput survival probability q = max(R, G, B).
Terminate if ξ > q; otherwise scale by 1/q. Unbiased because E[throughput] is preserved.

## Photometry

Illuminance (lux): E = I · cosθ / r²
- I [candela], r [metres], θ = angle between surface normal and direction to source.
- Inverse square law: doubling the distance quarters the illuminance.
