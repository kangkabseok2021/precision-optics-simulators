#pragma once

#include <algorithm>

namespace edge_ai_quadcopter {

// Single-axis PID controller. All state is plain float value members —
// no pointers, no heap — so instances can live in static/BSS storage
// and be constructed at compile time via the constexpr constructor.
class PidController {
 public:
  constexpr PidController(float kp, float ki, float kd, float ilim) noexcept
      : kp_(kp),
        ki_(ki),
        kd_(kd),
        ilim_(ilim),
        integral_(0.0f),
        prev_error_(0.0f) {}

  // u(t) = Kp*e(t) + Ki*integral(e) + Kd*de/dt, integral clamped to +-ilim_.
  float update(float error, float dt) noexcept {
    integral_ = std::clamp(integral_ + error * dt, -ilim_, ilim_);
    const float derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

  void reset() noexcept {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
  }

  float integral() const noexcept { return integral_; }
  float ilim() const noexcept { return ilim_; }

 private:
  float kp_;
  float ki_;
  float kd_;
  float ilim_;
  float integral_;
  float prev_error_;
};

}  // namespace edge_ai_quadcopter
