#!/usr/bin/env python3
"""Generate the tangent space normal map of the cracked sand texture.

Writes cracked_sand_normal.png next to this script, from cracked_sand.jpg.
The height of the surface is taken from the brightness of the texture: the
dark cracks are grooves and the bright plates stand up, with a slight blur
so single pixels of noise don't become bumps. Slopes are found with
wrap-around differences, so the map tiles like the texture.

The convention is that of generate_steel.py: x along the image columns, y
along the rows, z out of the surface.

usage: generate_sand_normal.py [strength]
"""

import os
import sys

import numpy as np
from PIL import Image

STRENGTH = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
here = os.path.dirname(os.path.abspath(__file__))

image = Image.open(os.path.join(here, "cracked_sand.jpg")).convert("RGB")
rgb = np.asarray(image, dtype=np.float64) / 255.0
# brightness of the sRGB values, Rec. 709 weights
height = rgb @ np.array([0.2126, 0.7152, 0.0722])

# Gaussian blur of 1.2 pixels, in the frequency domain so it wraps around
fy = np.fft.fftfreq(height.shape[0])[:, None]
fx = np.fft.fftfreq(height.shape[1])[None, :]
sigma = 1.2
kernel = np.exp(-2.0 * np.pi**2 * sigma**2 * (fx**2 + fy**2))
height = np.real(np.fft.ifft2(np.fft.fft2(height) * kernel))

dhdx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5
dhdy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5
normal = np.stack([-STRENGTH * dhdx, -STRENGTH * dhdy, np.ones_like(height)],
                  axis=-1)
normal /= np.linalg.norm(normal, axis=-1, keepdims=True)

pixels = np.clip(np.round((0.5 + 0.5 * normal) * 255.0), 0, 255)
Image.fromarray(pixels.astype(np.uint8), "RGB").save(
    os.path.join(here, "cracked_sand_normal.png"))
tilt = np.degrees(np.arccos(normal[..., 2]))
print(f"cracked_sand_normal.png: {image.size[0]} x {image.size[1]}, "
      f"tilt median {np.median(tilt):.1f}, 99th percentile "
      f"{np.percentile(tilt, 99):.1f} degrees")
