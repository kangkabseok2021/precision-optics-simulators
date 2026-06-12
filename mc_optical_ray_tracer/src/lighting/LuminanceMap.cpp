#include "lighting/LuminanceMap.h"

LuminanceMap::LuminanceMap(int width, int height)
    : width_(width), height_(height),
      accum_(width*height), count_(width*height, 0),
      locks_(width*height) {}

LuminanceMap::LuminanceMap(const LuminanceMap& o)
    : width_(o.width_), height_(o.height_),
      accum_(o.accum_), count_(o.count_),
      locks_(o.width_ * o.height_) {}

void LuminanceMap::add(int x, int y, const Vec3& radiance) {
    int idx = y * width_ + x;
    std::lock_guard<std::mutex> lock(locks_[idx]);
    accum_[idx] += radiance;
    count_[idx]++;
}

Vec3 LuminanceMap::get_mean(int x, int y) const noexcept {
    int idx = y * width_ + x;
    if (count_[idx] == 0) return Vec3{};
    return accum_[idx] * (1.0f / count_[idx]);
}
