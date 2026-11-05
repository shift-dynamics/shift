#!/usr/bin/env python3
"""Writes rally_stage.tiff, the elevation map of vehicle_terrain.json.

A 220 m x 80 m stretch of gravel road: crests and dips along the road, a
camber across it, and small bumps, flat around the start of the vehicle at
the origin. The map is 32 bit float samples on a 0.5 m grid, read by the
"basic" terrain with "coordinate_type": "sae": columns along x, rows along y,
and each sample is the z (down) of the surface, so elevations are negated.
Its terrain frame is at x = -20 m, y = -40 m, so the road (y = 0) runs along
row 80.
"""

import numpy as np
from PIL import Image

SPACING = 0.5               # m between samples
X0, Y0 = -20.0, -40.0       # world x, y of the first sample (terrain frame)
COLUMNS, ROWS = 441, 161    # 220 m along x, 80 m along y

x = X0 + SPACING * np.arange(COLUMNS)
y = Y0 + SPACING * np.arange(ROWS)
X, Y = np.meshgrid(x, y)    # rows along y, columns along x

# flat for the first 5 m, blending to the full stage over the next 15 m
ramp = np.clip((X - 5.0) / 15.0, 0.0, 1.0)
ramp = ramp * ramp * (3.0 - 2.0 * ramp)

# crests and dips along the road, and a long rise
crests = (0.6 * np.sin(2.0 * np.pi * X / 45.0) +
          0.25 * np.sin(2.0 * np.pi * X / 17.0 + 1.3) + 0.004 * X)
# camber across the road, falling to the right (+y), and hills off the road
camber = -0.03 * Y + 0.4 * np.sin(2.0 * np.pi * Y / 60.0) ** 2
# bumps a few meters across, from a fixed seed so the stage is reproducible
rng = np.random.default_rng(2017)
bumps = np.zeros_like(X)
for _ in range(400):
    cx = rng.uniform(5.0, 200.0)
    cy = rng.uniform(-6.0, 6.0)
    radius = rng.uniform(1.0, 3.0)
    height = rng.uniform(-0.08, 0.08)
    bumps += height * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / radius ** 2)

elevation = ramp * (crests + camber + bumps)
elevation -= elevation[np.abs(y).argmin(), np.abs(x).argmin()]  # 0 at origin

Image.fromarray((-elevation).astype(np.float32), mode="F").save(
    "rally_stage.tiff")
print("wrote rally_stage.tiff: %d x %d samples, elevation %.2f .. %.2f m" %
      (COLUMNS, ROWS, elevation.min(), elevation.max()))
