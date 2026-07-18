#!/usr/bin/env python3
"""Trains the 16-8-4-1 obstacle-detection MLP on synthetic LiDAR samples,
post-training-quantizes every layer to INT8, and regenerates
src/model_data.cc.

Deliberately dependency-light (NumPy only, no TensorFlow/Keras): the
network and the backprop loop are both hand-written so this script has
no heavy ML-framework dependency to install on a CI runner. Re-run it
whenever the training data or architecture changes; never hand-edit
model_data.cc.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

INPUT_SIZE = 16
HIDDEN1_SIZE = 8
HIDDEN2_SIZE = 4
OUTPUT_SIZE = 1

# Domain-motivated fixed range for the LiDAR input tensor: 0 to 8 m
# covers the sensor's full working range, so quantization never clips
# a legitimate reading even if it falls outside the training distribution.
INPUT_RANGE = (0.0, 8.0)
# Sigmoid's co-domain is (0, 1); fixing the output range to the nominal
# bound avoids a data-driven range that could clip near-0/near-1 cases
# not seen in the (finite) representative dataset.
OUTPUT_RANGE = (0.0, 1.0)

OBSTACLE_THRESHOLD_M = 1.0
QMIN, QMAX = -128, 127


def make_dataset(rng: np.random.Generator, n: int) -> tuple[np.ndarray, np.ndarray]:
    """20K synthetic 16-beam LiDAR scans; label=1 if any beam < 1.0 m.

    Built class-balanced by construction (half clear, half obstacle)
    rather than uniform-random-then-labeled: independently sampling all
    16 beams uniformly over the full sensor range makes P(min < 1.0m)
    ~87% for a 0.05-8m range, so a majority-class classifier already
    scores ~87% accuracy and the network never learns real separation.

    Obstacle beams form a contiguous 3-6-beam arc (a physical obstacle
    subtends multiple adjacent beam angles, not one isolated beam) —
    this also gives the 16-8-4-1 network enough signal per sample to
    actually learn the threshold; scattering 1-3 independently-chosen
    beams among 16 inputs plateaued at ~70% with this architecture.
    """
    n_obstacle = n // 2
    n_clear = n - n_obstacle

    clear = rng.uniform(OBSTACLE_THRESHOLD_M, INPUT_RANGE[1], size=(n_clear, INPUT_SIZE))

    obstacle = rng.uniform(OBSTACLE_THRESHOLD_M, INPUT_RANGE[1], size=(n_obstacle, INPUT_SIZE))
    arc_widths = rng.integers(3, 7, size=n_obstacle)
    arc_starts = rng.integers(0, INPUT_SIZE, size=n_obstacle)
    for row, (width, start) in enumerate(zip(arc_widths, arc_starts)):
        idx = [(start + k) % INPUT_SIZE for k in range(width)]
        obstacle[row, idx] = rng.uniform(INPUT_RANGE[0] + 0.05, OBSTACLE_THRESHOLD_M, size=width)

    x = np.concatenate([clear, obstacle], axis=0)
    y = np.concatenate(
        [np.zeros((n_clear, 1)), np.ones((n_obstacle, 1))], axis=0
    )
    perm = rng.permutation(n)
    return x[perm], y[perm]


def sigmoid(z: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-z))


def train(
    rng: np.random.Generator, x: np.ndarray, y: np.ndarray, epochs: int, lr: float
) -> dict[str, np.ndarray]:
    """Hand-written full-batch backprop for the 16-8-4-1 MLP."""
    w1 = rng.normal(0, np.sqrt(2.0 / INPUT_SIZE), (INPUT_SIZE, HIDDEN1_SIZE))
    b1 = np.zeros(HIDDEN1_SIZE)
    w2 = rng.normal(0, np.sqrt(2.0 / HIDDEN1_SIZE), (HIDDEN1_SIZE, HIDDEN2_SIZE))
    b2 = np.zeros(HIDDEN2_SIZE)
    w3 = rng.normal(0, np.sqrt(2.0 / HIDDEN2_SIZE), (HIDDEN2_SIZE, OUTPUT_SIZE))
    b3 = np.zeros(OUTPUT_SIZE)

    # Normalize the raw meter readings into [0, 1] for training stability;
    # the *deployed* INT8 model still quantizes directly from raw meters
    # via kInputScale/kInputZeroPoint (see InferenceEngine::run).
    x_norm = (x - INPUT_RANGE[0]) / (INPUT_RANGE[1] - INPUT_RANGE[0])
    n = x_norm.shape[0]

    for epoch in range(epochs):
        z1 = x_norm @ w1 + b1
        h1 = np.maximum(0.0, z1)
        z2 = h1 @ w2 + b2
        h2 = np.maximum(0.0, z2)
        z3 = h2 @ w3 + b3
        pred = sigmoid(z3)

        d_z3 = (pred - y) / n
        d_w3 = h2.T @ d_z3
        d_b3 = d_z3.sum(axis=0)

        d_h2 = d_z3 @ w3.T
        d_z2 = d_h2 * (z2 > 0)
        d_w2 = h1.T @ d_z2
        d_b2 = d_z2.sum(axis=0)

        d_h1 = d_z2 @ w2.T
        d_z1 = d_h1 * (z1 > 0)
        d_w1 = x_norm.T @ d_z1
        d_b1 = d_z1.sum(axis=0)

        w1 -= lr * d_w1
        b1 -= lr * d_b1
        w2 -= lr * d_w2
        b2 -= lr * d_b2
        w3 -= lr * d_w3
        b3 -= lr * d_b3

        if epoch % 300 == 0 or epoch == epochs - 1:
            loss = -np.mean(y * np.log(pred + 1e-9) + (1 - y) * np.log(1 - pred + 1e-9))
            acc = np.mean((pred > 0.5) == y)
            print(f"  epoch {epoch:4d}  loss={loss:.4f}  acc={acc:.4f}")

    return {"w1": w1, "b1": b1, "w2": w2, "b2": b2, "w3": w3, "b3": b3}


def quantize_activation_range(lo: float, hi: float) -> tuple[float, int]:
    """Asymmetric INT8: real = scale * (q - zero_point)."""
    scale = (hi - lo) / (QMAX - QMIN)
    zero_point = int(round(QMIN - lo / scale))
    zero_point = max(QMIN, min(QMAX, zero_point))
    return scale, zero_point


def quantize_weights(w: np.ndarray) -> tuple[np.ndarray, float]:
    """Symmetric per-tensor INT8: weight_zero_point is always 0."""
    scale = max(np.abs(w).max(), 1e-8) / 127.0
    q = np.clip(np.round(w / scale), -127, 127).astype(np.int8)
    return q, scale


def quantize_bias(b: np.ndarray, input_scale: float, weight_scale: float) -> np.ndarray:
    bias_scale = input_scale * weight_scale
    return np.round(b / bias_scale).astype(np.int32)


def format_c_array(name: str, values: np.ndarray, ctype: str) -> str:
    flat = ", ".join(str(int(v)) for v in values.flatten())
    return f"const {ctype} {name}[{values.size}] = {{{flat}}};"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=20_000)
    parser.add_argument("--epochs", type=int, default=3000)
    parser.add_argument("--lr", type=float, default=0.3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "src" / "model_data.cc",
    )
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    x, y = make_dataset(rng, args.samples)

    print(f"Training on {args.samples} synthetic LiDAR samples...")
    params = train(rng, x, y, args.epochs, args.lr)

    # Calibrate hidden-layer activation ranges on a representative subset
    # (post-ReLU, so min is always 0; max comes from the actual data).
    x_norm = (x - INPUT_RANGE[0]) / (INPUT_RANGE[1] - INPUT_RANGE[0])
    calib = x_norm[: min(2000, x_norm.shape[0])]
    z1 = calib @ params["w1"] + params["b1"]
    h1 = np.maximum(0.0, z1)
    z2 = h1 @ params["w2"] + params["b2"]
    h2 = np.maximum(0.0, z2)

    input_scale, input_zp = quantize_activation_range(*INPUT_RANGE)
    h1_scale, h1_zp = quantize_activation_range(0.0, max(h1.max() * 1.05, 1e-3))
    h2_scale, h2_zp = quantize_activation_range(0.0, max(h2.max() * 1.05, 1e-3))
    out_scale, out_zp = quantize_activation_range(*OUTPUT_RANGE)

    w1_q, w1_scale = quantize_weights(params["w1"])
    w2_q, w2_scale = quantize_weights(params["w2"])
    w3_q, w3_scale = quantize_weights(params["w3"])

    b1_q = quantize_bias(params["b1"], input_scale, w1_scale)
    b2_q = quantize_bias(params["b2"], h1_scale, w2_scale)
    b3_q = quantize_bias(params["b3"], h2_scale, w3_scale)

    # Row-major [input][output] layout, matching InferenceEngine::DenseLayer
    # indexing weights[i * output_size + j].
    lines = [
        "// Generated by scripts/generate_model.py — DO NOT EDIT BY HAND.",
        "// Regenerate: python3 scripts/generate_model.py",
        "",
        '#include "edge_ai_quadcopter/model_data.h"',
        "",
        "namespace edge_ai_quadcopter::model {",
        "",
        f"const float kInputScale = {input_scale:.10f}f;",
        f"const int32_t kInputZeroPoint = {input_zp};",
        "",
        format_c_array("kLayer1Weights", w1_q, "int8_t"),
        format_c_array("kLayer1Bias", b1_q, "int32_t"),
        f"const float kLayer1WeightScale = {w1_scale:.10f}f;",
        f"const float kLayer1OutputScale = {h1_scale:.10f}f;",
        f"const int32_t kLayer1OutputZeroPoint = {h1_zp};",
        "",
        format_c_array("kLayer2Weights", w2_q, "int8_t"),
        format_c_array("kLayer2Bias", b2_q, "int32_t"),
        f"const float kLayer2WeightScale = {w2_scale:.10f}f;",
        f"const float kLayer2OutputScale = {h2_scale:.10f}f;",
        f"const int32_t kLayer2OutputZeroPoint = {h2_zp};",
        "",
        format_c_array("kLayer3Weights", w3_q, "int8_t"),
        format_c_array("kLayer3Bias", b3_q, "int32_t"),
        f"const float kLayer3WeightScale = {w3_scale:.10f}f;",
        f"const float kLayer3OutputScale = {out_scale:.10f}f;",
        f"const int32_t kLayer3OutputZeroPoint = {out_zp};",
        "",
        "}  // namespace edge_ai_quadcopter::model",
        "",
    ]

    args.out.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
