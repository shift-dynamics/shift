#!/usr/bin/env python3
"""Write examples/terramechanics/track_sand.tiff, the undisturbed surface of the sand in
examples/terramechanics/track_frame_scm.json.

Loose sand is never quite flat: a few gentle swells a meter or two long and
smaller hummocks of a few decimeters, a centimeter or so high. The samples
are 32 bit floats, the z of the surface (m, +z down), on a grid of SPACING
from the origin of the terrain frame, which the model places at
(X0, Y0) so that the map covers the grid of the terrain with a margin.

Under the track where it starts the variation is reduced to a few
millimeters, so that the track starts on the ground and does not drop or
jump at the start.

The seed is fixed, so the file is reproducible.

usage: make_track_terrain.py
"""

import os

import numpy as np
from PIL import Image

SEED = 20261108
SPACING = 0.05                 # m, of the samples
X0, Y0 = -1.0, -1.0            # m, the terrain frame in the model
LENGTH, WIDTH = 6.4, 2.0       # m, covered by the map
START_X = (-0.35, 3.55)        # m, under the track at the start
START_HALF_Y = 0.35
START_FACTOR = 0.3             # of the variation under the start
BLEND = 0.5                    # m, from START_FACTOR to all of it

# amplitude (m), wavelength (m), heading (deg), phase (rad)
SWELLS = [(0.006, 2.6, 15.0, 0.4),
          (0.004, 1.7, -35.0, 2.1),
          (0.002, 1.1, 70.0, 4.0)]
HUMMOCK_SIGMA = 0.12           # m, of the smoothing of the hummocks
HUMMOCK_HEIGHT = 0.003         # m, standard deviation

here = os.path.dirname(os.path.abspath(__file__))
rng = np.random.default_rng(SEED)

nx = int(round(LENGTH / SPACING)) + 1
ny = int(round(WIDTH / SPACING)) + 1
x = X0 + SPACING * np.arange(nx)
y = Y0 + SPACING * np.arange(ny)
X, Y = np.meshgrid(x, y)

elevation = np.zeros_like(X)
for amplitude, wavelength, heading, phase in SWELLS:
    along = X * np.cos(np.radians(heading)) + Y * np.sin(np.radians(heading))
    elevation += amplitude * np.sin(2.0 * np.pi * along / wavelength + phase)

# hummocks: white noise smoothed by a Gaussian, wrapping around
fy = np.fft.fftfreq(ny, SPACING)[:, None]
fx = np.fft.fftfreq(nx, SPACING)[None, :]
kernel = np.exp(-2.0 * np.pi**2 * HUMMOCK_SIGMA**2 * (fx**2 + fy**2))
hummocks = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal(X.shape)) *
                                kernel))
elevation += HUMMOCK_HEIGHT * (hummocks - hummocks.mean()) / hummocks.std()

# less of it under the start, by the distance outside the start's rectangle
dx = np.maximum(np.maximum(START_X[0] - X, X - START_X[1]), 0.0)
dy = np.maximum(np.abs(Y) - START_HALF_Y, 0.0)
blend = np.clip(np.hypot(dx, dy) / BLEND, 0.0, 1.0)
blend = blend * blend * (3.0 - 2.0 * blend)
elevation *= START_FACTOR + (1.0 - START_FACTOR) * blend

path = os.path.join(here, "..", "examples", "terramechanics",
                    "track_sand.tiff")
# a sample is the z of the surface, and +z is down
Image.fromarray((-elevation).astype(np.float32), mode="F").save(path)
print("wrote %s: %d x %d samples, elevation %.1f .. %.1f mm"
      % (os.path.normpath(path), nx, ny, 1000.0 * elevation.min(),
         1000.0 * elevation.max()))
