#pragma once
#include "geometry/Vec3.h"
#include <vector>
#include <mutex>

class LuminanceMap {
public:
    LuminanceMap(int width, int height);
    LuminanceMap(const LuminanceMap& o);

    void add(int x, int y, const Vec3& radiance);
    Vec3 get_mean(int x, int y) const noexcept;

    int width()  const noexcept { return width_; }
    int height() const noexcept { return height_; }

private:
    int width_, height_;
    std::vector<Vec3>  accum_;
    std::vector<int>   count_;
    mutable std::vector<std::mutex> locks_;
};
