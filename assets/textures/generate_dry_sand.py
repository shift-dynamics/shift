#!/usr/bin/env python3
"""Generate the texture of loose dry sand and its normal map.

Writes dry_sand.jpg and dry_sand_normal.jpg next to this script, as JPEG
like the other textures, which are compiled into the viewer. The sand is
loose and without cohesion (the dry_sand soil of Wong's tables): fine grains
with no crust or cracks, unlike cracked_sand.jpg. The image is meant to tile
0.5 m of ground, so a pixel of the 1024 x 1024 image is about 0.5 mm, the
size of a medium grain.

The color is that of quartz sand, a warm beige, with:
- grains: noise at the scale of a pixel or two, mostly light quartz with a
  few darker mineral grains;
- clumps: noise of a few millimeters, where grains of one shade gather;
- tone: a faint variation over a few centimeters, so tiles don't show.

All noise is filtered in the frequency domain, so the image wraps around and
tiles without seams. The seed is fixed, so the files are reproducible.

The normal map follows the convention of generate_steel.py: x along the image
columns, y along the rows, z out of the surface. Its height is the grains and
clumps, not the color, so dark grains are not taken for pits, and lumps and
undulations of 5 to 30 mm, so the surface does not look smooth from a few
meters away.

usage: generate_dry_sand.py
"""

import os

import numpy as np
from PIL import Image

SIZE = 1024
SEED = 20261108
here = os.path.dirname(os.path.abspath(__file__))
rng = np.random.default_rng(SEED)

fy = np.fft.fftfreq(SIZE)[:, None]
fx = np.fft.fftfreq(SIZE)[None, :]
radius2 = fx**2 + fy**2


def noise(sigma):
    """White noise blurred by a Gaussian of sigma pixels, wrapping around,
    scaled to unit standard deviation"""
    white = rng.standard_normal((SIZE, SIZE))
    kernel = np.exp(-2.0 * np.pi**2 * sigma**2 * radius2)
    field = np.real(np.fft.ifft2(np.fft.fft2(white) * kernel))
    return (field - field.mean()) / field.std()


grain = noise(0.7)
clump = noise(4.0)
tone = noise(60.0)
# a few dark mineral grains, about 6 % of the area
dark = np.clip((noise(0.9) - 1.55) * 2.0, 0.0, 1.0)

# linear reflectance of quartz sand, then the shades
base = np.array([0.52, 0.40, 0.25])
shade = 1.0 + 0.10 * grain + 0.06 * clump + 0.05 * tone
color = base[None, None, :] * shade[:, :, None]
dark_color = np.array([0.16, 0.13, 0.10])
color = color * (1.0 - dark[:, :, None]) + dark_color * dark[:, :, None]
# darker sand is browner, not greener: less blue and more red where it is
# shaded, as damp or iron stained grains are
darker = np.clip(1.0 - shade, 0.0, None)
color[:, :, 0] *= 1.0 + 0.6 * darker
color[:, :, 2] *= 1.0 - 0.8 * darker
color = np.clip(color, 0.0, 1.0)

# linear to sRGB
srgb = np.where(color <= 0.0031308, 12.92 * color,
                1.055 * np.power(color, 1.0 / 2.4) - 0.055)
Image.fromarray(np.round(srgb * 255.0).astype(np.uint8), "RGB").save(
    os.path.join(here, "dry_sand.jpg"), quality=92, optimize=True)

# height: grains on clumps, on lumps and undulations of the loose surface
# (5 to 30 mm), large enough to show from a few meters away, where the
# grains average out; in pixels
lump = noise(10.0)
undulation = noise(32.0)
height = 0.6 * grain + 1.2 * clump + 10.0 * lump + 24.0 * undulation
STRENGTH = 0.35
dhdx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5
dhdy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5
normal = np.stack([-STRENGTH * dhdx, -STRENGTH * dhdy, np.ones_like(height)],
                  axis=-1)
normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
Image.fromarray(np.round((normal * 0.5 + 0.5) * 255.0).astype(np.uint8),
                "RGB").save(os.path.join(here, "dry_sand_normal.jpg"),
                            quality=95, optimize=True)
