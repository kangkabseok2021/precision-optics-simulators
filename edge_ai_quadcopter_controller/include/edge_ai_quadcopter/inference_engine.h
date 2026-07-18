#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <new>

#include "edge_ai_quadcopter/model_data.h"

namespace edge_ai_quadcopter {

// Hand-rolled, TFLM-style INT8 inference engine for the 16-beam-lidar
// obstacle-probability MLP (16 -> 8 -> 4 -> 1). This mirrors the real
// TensorFlow Lite Micro architecture — a fixed-size static arena holding
// runtime activation tensors, constructed in place via placement-new, so
// there is no heap allocation once init() has run — but it is our own
// small interpreter, not the upstream TFLM library (which has no CMake
// build suitable for a portable cross-compiled target; see README.md).
class InferenceEngine {
 public:
  // Deliberately generous relative to the ~32 bytes of int8/int32
  // activation buffers actually used per inference, mirroring how real
  // TFLM deployments provision headroom in the static tensor arena.
  static constexpr std::size_t kArenaSize = 32 * 1024;

  InferenceEngine() noexcept : initialized_(false) {}

  // Placement-constructs the activation-buffer object into the static
  // arena. Idempotent; safe to call once at startup before any run().
  void init() noexcept {
    activations_ = new (arena_.data()) Activations();
    initialized_ = true;
  }

  // Runs the full quantize -> layer1 -> layer2 -> layer3 -> dequantize
  // pipeline over `n` lidar beam readings (meters) and returns the
  // obstacle probability in [0, 1]. Returns NaN if init() has not run
  // or `n` does not match the model's expected input size.
  float run(const float* input, std::size_t n) const noexcept {
    if (!initialized_ || n != model::kInputSize) {
      return std::nanf("");
    }

    for (std::size_t i = 0; i < model::kInputSize; ++i) {
      activations_->input_q[i] = Quantize(input[i], model::kInputScale,
                                           model::kInputZeroPoint);
    }

    DenseLayer(activations_->input_q.data(), model::kInputSize,
               model::kLayer1Weights, model::kLayer1Bias,
               model::kLayer1WeightScale, model::kInputScale,
               model::kInputZeroPoint, model::kLayer1OutputScale,
               model::kLayer1OutputZeroPoint, /*apply_relu=*/true,
               activations_->hidden1_q.data(), model::kHidden1Size);

    DenseLayer(activations_->hidden1_q.data(), model::kHidden1Size,
               model::kLayer2Weights, model::kLayer2Bias,
               model::kLayer2WeightScale, model::kLayer1OutputScale,
               model::kLayer1OutputZeroPoint, model::kLayer2OutputScale,
               model::kLayer2OutputZeroPoint, /*apply_relu=*/true,
               activations_->hidden2_q.data(), model::kHidden2Size);

    int8_t output_q = 0;
    DenseLayer(activations_->hidden2_q.data(), model::kHidden2Size,
               model::kLayer3Weights, model::kLayer3Bias,
               model::kLayer3WeightScale, model::kLayer2OutputScale,
               model::kLayer2OutputZeroPoint, model::kLayer3OutputScale,
               model::kLayer3OutputZeroPoint, /*apply_relu=*/false,
               &output_q, model::kOutputSize, /*apply_sigmoid=*/true);

    return Dequantize(output_q, model::kLayer3OutputScale,
                       model::kLayer3OutputZeroPoint);
  }

  bool initialized() const noexcept { return initialized_; }

 private:
  // Runtime activation tensors — the actual payload placement-new'd into
  // the static arena. Sized for the fixed 16-8-4-1 topology.
  struct Activations {
    std::array<int8_t, model::kInputSize> input_q{};
    std::array<int8_t, model::kHidden1Size> hidden1_q{};
    std::array<int8_t, model::kHidden2Size> hidden2_q{};
  };

  static int8_t Quantize(float real_value, float scale,
                         int32_t zero_point) noexcept {
    const int32_t q =
        static_cast<int32_t>(std::lround(real_value / scale)) + zero_point;
    return static_cast<int8_t>(std::clamp<int32_t>(q, -128, 127));
  }

  static float Dequantize(int8_t q, float scale, int32_t zero_point) noexcept {
    return scale * static_cast<float>(static_cast<int32_t>(q) - zero_point);
  }

  // Computes one fully-connected layer entirely in the quantized domain:
  // acc[j] = bias[j] + sum_i (input_q[i] - input_zero_point) * weight[i, j]
  // then rescales to real units (input_scale * weight_scale), applies the
  // optional activation in real units, and requantizes to the layer's
  // output int8 tensor using output_scale/output_zero_point.
  static void DenseLayer(const int8_t* input_q, std::size_t input_size,
                         const int8_t* weights, const int32_t* bias,
                         float weight_scale, float input_scale,
                         int32_t input_zero_point, float output_scale,
                         int32_t output_zero_point, bool apply_relu,
                         int8_t* output_q, std::size_t output_size,
                         bool apply_sigmoid = false) noexcept {
    const float rescale = input_scale * weight_scale;
    for (std::size_t j = 0; j < output_size; ++j) {
      int32_t acc = bias[j];
      for (std::size_t i = 0; i < input_size; ++i) {
        acc += (static_cast<int32_t>(input_q[i]) - input_zero_point) *
               static_cast<int32_t>(weights[i * output_size + j]);
      }
      float real_value = static_cast<float>(acc) * rescale;
      if (apply_relu) {
        real_value = std::max(0.0f, real_value);
      }
      if (apply_sigmoid) {
        real_value = 1.0f / (1.0f + std::exp(-real_value));
      }
      output_q[j] = Quantize(real_value, output_scale, output_zero_point);
    }
  }

  alignas(16) std::array<uint8_t, kArenaSize> arena_{};
  Activations* activations_ = nullptr;
  bool initialized_;
};

}  // namespace edge_ai_quadcopter
