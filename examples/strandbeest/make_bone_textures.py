#!/usr/bin/env python3
"""Write a tileable bone texture set for metallic-roughness PBR, for the
links of strandbeest.json (applied by triplanar projection).

Writes, next to this script:
  bone_base_color.png  sRGB base color: ivory, mottled with yellowed and
                       brownish stains, with dark pores
  bone_roughness.png   linear roughness: a satin surface, rough in the pores,
                       glossy where worn smooth
  bone_normal.png      tangent space normal map of the pores, the fine
                       fibrous grain and the low, rolling relief

The noise is filtered in the frequency domain, so every map tiles seamlessly.
The grain runs faintly along the texture's x axis.

usage: make_bone_textures.py [size]
"""

import os
import sys

import numpy as np
from PIL import Image

SIZE = int(sys.argv[1]) if len(sys.argv) > 1 else 1024
rng = np.random.default_rng(20261113)
fx = np.fft.fftfreq(SIZE)[None, :]
fy = np.fft.fftfreq(SIZE)[:, None]


def filtered_noise(sigma_x, sigma_y):
    """white noise low-pass filtered by an anisotropic Gaussian (sigmas in
    texels), normalized to zero mean and unit variance"""
    spectrum = np.fft.fft2(rng.standard_normal((SIZE, SIZE)))
    kernel = np.exp(-2.0 * np.pi**2 * ((fx * sigma_x)**2 + (fy * sigma_y)**2))
    noise = np.real(np.fft.ifft2(spectrum * kernel))
    return (noise - noise.mean()) / noise.std()


def smoothstep(a, b, x):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def linear_to_srgb(c):
    c = np.clip(c, 0.0, 1.0)
    return np.where(c <= 0.0031308, 12.92 * c, 1.055 * c**(1 / 2.4) - 0.055)


def srgb_to_linear(c):
    c = np.asarray(c, dtype=float)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055)**2.4)


def save(name, values, mode):
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
    image = np.clip(np.round(values * 255.0), 0, 255).astype(np.uint8)
    Image.fromarray(image, mode).save(path, optimize=True)
    print("wrote", path)


# stains: large soft blotches (yellowing) and smaller, sharper brown ones
yellowing = smoothstep(-0.3, 1.4, filtered_noise(90.0, 90.0))
brown = smoothstep(1.0, 2.2, filtered_noise(28.0, 28.0)) * \
    smoothstep(-0.5, 0.8, filtered_noise(120.0, 120.0))
# relief: low rolling bumps and a fine fibrous grain along x
rolling = filtered_noise(40.0, 40.0)
grain = 0.7 * filtered_noise(14.0, 1.6) + 0.3 * filtered_noise(4.0, 0.8)
grain /= grain.std()
# pores: the peaks of fine noise, thresholded into small pits, denser where
# the bone is spongier (a slow mask)
sponginess = smoothstep(-1.2, 1.2, filtered_noise(70.0, 70.0))
pore_noise = filtered_noise(1.6, 1.6)
pores = smoothstep(2.1 - 0.5 * sponginess, 2.9 - 0.5 * sponginess, pore_noise)
# worn, polished patches (glossier, a little lighter)
worn = smoothstep(0.6, 1.8, filtered_noise(55.0, 55.0))

# base color: ivory, toward a straw yellow where yellowed, a tea brown in the
# stains, darker in the pores and in the hollows of the relief
ivory = srgb_to_linear([0.90, 0.86, 0.76])
straw = srgb_to_linear([0.84, 0.74, 0.52])
tea = srgb_to_linear([0.55, 0.42, 0.27])
pore = srgb_to_linear([0.30, 0.22, 0.14])
color = ivory[None, None, :] * np.ones((SIZE, SIZE, 1))
color = color + (straw - ivory)[None, None, :] * (0.7 * yellowing)[:, :, None]
color = color + (tea - color) * (0.75 * brown)[:, :, None]
shade = 1.0 + 0.035 * grain + 0.05 * rolling + 0.05 * worn
color *= shade[:, :, None]
color = color + (pore - color) * (0.85 * pores)[:, :, None]
save("bone_base_color.png", linear_to_srgb(color), "RGB")

# roughness: satin bone, rougher in the pores and the stains, glossy where
# worn
roughness = (0.58 + 0.04 * grain + 0.25 * pores + 0.06 * brown -
             0.22 * worn)
save("bone_roughness.png", np.clip(roughness, 0.25, 0.95), "L")

# normal map: pores are pits, the grain fine ridges along x, the rolling
# relief broad and low
height = 0.5 * rolling / 6.0 + 0.10 * grain - 1.6 * pores
strength = 1.0
dhdx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5
dhdy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5
normal = np.stack([-strength * dhdx, -strength * dhdy, np.ones_like(height)],
                  axis=-1)
normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
save("bone_normal.png", 0.5 + 0.5 * normal, "RGB")
