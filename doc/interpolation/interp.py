#!/usr/bin/env python

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

f11 = 0.5
f12 = 0.7
f21 = 0.6
f22 = 0.4

fig = plt.figure()
ax = fig.gca(projection='3d')

x = [0.0, 0.0, 0.5, 0.5]
y = [0.0, 0.6, 0.0, 0.6]
z = [0.5, 0.7, 0.6, 0.4]

ax.scatter(x, y, z)

# plot bounding line segments
ax.plot([x[0], x[1]], [y[0], y[1]], [z[0], z[1]], color='k', alpha=0.5)
ax.plot([x[0], x[2]], [y[0], y[2]], [z[0], z[2]], color='k', alpha=0.5)
ax.plot([x[1], x[3]], [y[1], y[3]], [z[1], z[3]], color='k', alpha=0.5)
ax.plot([x[2], x[3]], [y[2], y[3]], [z[2], z[3]], color='k', alpha=0.5)

X = np.arange(0.0, 0.5 + 0.05, 0.05)
Y = np.arange(0.0, 0.6 + 0.05, 0.05)
X, Y = np.meshgrid(X, Y)

x1 = 0.0
x2 = 0.5
y1 = 0.0
y2 = 0.6

# plot interpolated bilinear surface
Z = np.zeros(X.shape)
for i in range(Z.shape[0]):
    for j in range(Z.shape[1]):
        u = (X[i, j] - x1) / (x2 - x1)
        v = (Y[i, j] - y1) / (y2 - y1)
        a11 = f11
        a21 = f21 - f11
        a12 = f12 - f11
        a22 = f11 - f21 - f12 + f22
        Z[i, j] = a11 + a21 * u + a12 * v + a22 * u * v

surf = ax.plot_surface(X, Y, Z, linewidth=0, antialiased=False, alpha=0.5)

max_range = np.array(
    [X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()]
    ).max() / 2.0

mid_x = (X.max() + X.min()) * 0.5
mid_y = (Y.max() + Y.min()) * 0.5
mid_z = (Z.max() + Z.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# plot normal vector at x=0.25, y = 0.3
n_x1 = 0.25
n_y1 = 0.3
u = (n_x1 - x1) / (x2 - x1)
v = (n_y1 - y1) / (y2 - y1)
a11 = f11
a21 = f21 - f11
a12 = f12 - f11
a22 = f11 - f21 - f12 + f22
n_z1 = a11 + a21 * u + a12 * v + a22 * u * v

n_x2 = -(a21 + a22 * v) / (x2 - x1)
n_y2 = -(a12 + a22 * u) / (y2 - y1)
n_z2 = 1.0

norm = np.sqrt(n_x2**2 + n_y2**2 + n_z2**2)

n_x2 /= norm
n_y2 /= norm
n_z2 /= norm

ax.plot([n_x1, n_x1 + n_x2], [n_y1, n_y1 + n_y2], [n_z1, n_z1 + n_z2])

# draw a line that intersects the surface

r_cx = 0.3
r_cy = 0.35
r_cz = 0.8

r_px = 0.2
r_py = 0.3
r_pz = 0.3

ax.plot([r_cx, r_px], [r_cy, r_py], [r_cz, r_pz])

# solve for intersection point of line with surface
b0 = r_cz * (x2 - x1) * (y2 - y1)
b1 = (r_pz - r_cz) * (x2 - x1) * (y2 - y1)
b2 = x2 - r_cx
b3 = r_cx - r_px
b4 = r_cx - x1
b5 = y2 - r_cy
b6 = r_cy - r_py
b7 = r_cy - y1

# note we can save some multiplies because we already computed some
# of these coefficients above in a21, a12 and a22
c2 = b3 * b6 * (f11 - f21 - f12 + f22)
c1 = b3 * (b5 * (f11 - f21) + b7 * (f12 - f22)) + \
     b6 * (b2 * (f11 - f12) + b4 * (f21 - f22)) - b1
c0 = b5 * (b2 * f11 + b4 * f21) + b7 * (b2 * f12 + b4 * f22) - b0

t_1 = (-c1 + np.sqrt(c1**2 - 4 * c2 * c0)) / (2 * c2)
t_2 = (-c1 - np.sqrt(c1**2 - 4 * c2 * c0)) / (2 * c2)

print(t_1, t_2)

# need to check both roots to verify that t is (0 <= t <= ~1.5)
# in this example t_1 is 0.5
r_tx = r_cx + (r_px - r_cx) * t_1
r_ty = r_cy + (r_py - r_cy) * t_1
r_tz = r_cz + (r_pz - r_cz) * t_1

ax.scatter(r_tx, r_ty, r_tz)

plt.show()
