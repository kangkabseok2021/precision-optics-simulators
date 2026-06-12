#pragma once
#include <cmath>

struct Vec3 {
    float x{0.f}, y{0.f}, z{0.f};

    Vec3 operator+(const Vec3& o) const noexcept { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const noexcept { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)       const noexcept { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t)       const noexcept { return {x/t, y/t, z/t}; }
    Vec3 operator*(const Vec3& o) const noexcept { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator-()              const noexcept { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) noexcept { x+=o.x; y+=o.y; z+=o.z; return *this; }

    float dot(const Vec3& o)  const noexcept { return x*o.x + y*o.y + z*o.z; }
    Vec3  cross(const Vec3& o) const noexcept {
        return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x};
    }
    float length()    const noexcept { return std::sqrt(x*x+y*y+z*z); }
    float length_sq() const noexcept { return x*x+y*y+z*z; }
    Vec3  normalized() const noexcept { float l=length(); return l > 0.0f ? Vec3{x/l, y/l, z/l} : Vec3{}; }
    float operator[](int i) const noexcept { return i==0?x:i==1?y:z; }
    float& operator[](int i) noexcept {
        if (i==0) return x; if (i==1) return y; return z;
    }
};

inline Vec3 operator*(float t, const Vec3& v) noexcept { return v*t; }
