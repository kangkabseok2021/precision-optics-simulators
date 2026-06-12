#!/usr/bin/env python3
"""Load render.ppm, compute per-pixel luminance, output histogram and stats."""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_ppm(path: str) -> np.ndarray:
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P3"
        # Skip potential comment lines
        line = f.readline().strip()
        while line.startswith(b"#"):
            line = f.readline().strip()
        w, h = map(int, line.split())
        assert int(f.readline()) == 255
        data = np.fromstring(f.read(), dtype=np.uint8, sep=" ")
    return data.reshape(h, w, 3).astype(np.float32) / 255.0


def srgb_to_linear(img: np.ndarray) -> np.ndarray:
    return img ** 2.0  # undo gamma 2 applied in main.cpp


def luminance(linear: np.ndarray) -> np.ndarray:
    # ITU-R BT.709 coefficients
    return 0.2126 * linear[..., 0] + 0.7152 * linear[..., 1] + 0.0722 * linear[..., 2]


def main(ppm_path: str = "render.ppm") -> None:
    img    = load_ppm(ppm_path)
    linear = srgb_to_linear(img)
    lum    = luminance(linear)

    print(f"Image: {img.shape[1]}x{img.shape[0]}")
    print(f"Luminance  min={lum.min():.4f}  max={lum.max():.4f}  mean={lum.mean():.4f}")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].imshow(np.clip(img, 0, 1))
    axes[0].set_title("Render (gamma 2)")
    axes[0].axis("off")

    axes[1].hist(lum.ravel(), bins=64, color="steelblue", edgecolor="none")
    axes[1].set_xlabel("Luminance")
    axes[1].set_ylabel("Pixel count")
    axes[1].set_title("Luminance Distribution")

    out = ppm_path.replace(".ppm", "_analysis.png")
    plt.tight_layout()
    plt.savefig(out, dpi=150)
    print(f"Saved {out}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "render.ppm")
