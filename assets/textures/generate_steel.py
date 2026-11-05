#!/usr/bin/env python3
"""Generate a tileable brushed steel texture set for metallic-roughness PBR.

Writes, next to this script:
  steel_base_color.png  sRGB base color (for a metal, its reflectance F0)
  steel_roughness.png   linear roughness, brushed streaks plus wear
  steel_normal.png      tangent space normal map of the brushing grooves

The noise is filtered in the frequency domain, so every map tiles seamlessly.
Brushing runs along the texture's x axis.

usage: generate_steel.py [size]
"""

import os
import sys

import numpy as np
from PIL import Image

SIZE = int(sys.argv[1]) if len(sys.argv) > 1 else 1024
rng = np.random.default_rng(20261031)
fx = np.fft.fftfreq(SIZE)[None, :]
fy = np.fft.fftfreq(SIZE)[:, None]


def filtered_noise(sigma_x, sigma_y):
    """white noise low-pass filtered by an anisotropic Gaussian (sigmas in
    texels), normalized to zero mean and unit variance"""
    spectrum = np.fft.fft2(rng.standard_normal((SIZE, SIZE)))
    kernel = np.exp(-2.0 * np.pi**2 * ((fx * sigma_x)**2 + (fy * sigma_y)**2))
    noise = np.real(np.fft.ifft2(spectrum * kernel))
    return (noise - noise.mean()) / noise.std()


def linear_to_srgb(c):
    c = np.clip(c, 0.0, 1.0)
    return np.where(c <= 0.0031308, 12.92 * c, 1.055 * c**(1 / 2.4) - 0.055)


def save(name, values, mode):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
    image = np.clip(np.round(values * 255.0), 0, 255).astype(np.uint8)
    Image.fromarray(image, mode).save(path, optimize=True)
    print("wrote", path)


# brushing: long, thin streaks along x at a few scales
streaks = (0.6 * filtered_noise(200.0, 0.6) + 0.3 * filtered_noise(60.0, 1.5) +
           0.1 * filtered_noise(15.0, 0.5))
streaks /= streaks.std()
# wear: soft, isotropic blotches
wear = filtered_noise(40.0, 40.0)
wear_mask = np.clip(0.5 + 0.35 * wear, 0.0, 1.0)

# base color: steel reflectance (linear F0 about 0.56), slightly darker where
# worn, with a faint streak modulation
f0 = np.array([0.56, 0.57, 0.58])
shade = 1.0 + 0.04 * streaks - 0.10 * (wear_mask - 0.5)
base_color = linear_to_srgb(f0[None, None, :] * shade[:, :, None])
save("steel_base_color.png", base_color, "RGB")

# roughness: brushed metal is fairly glossy across the grooves, rougher in
# worn areas
roughness = np.clip(0.32 + 0.06 * streaks + 0.18 * (wear_mask - 0.5), 0.15,
                    0.7)
save("steel_roughness.png", roughness, "L")

# normal map: grooves from the fine streaks, height varies across the
# brushing direction (y), so normals tilt in y
height = 0.6 * filtered_noise(80.0, 0.7) + 0.4 * filtered_noise(20.0, 0.5)
strength = 0.6
dhdx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5
dhdy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5
normal = np.stack([-strength * dhdx, -strength * dhdy, np.ones_like(height)],
                  axis=-1)
normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
save("steel_normal.png", 0.5 + 0.5 * normal, "RGB")
