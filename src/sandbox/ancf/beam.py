#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.optimize as opt


def mix(a, b, t):
    return (1 - t) * a + t * b


def cubic_bezier(a, b, c, d, t):
    e = mix(a, b, t)
    f = mix(b, c, t)
    g = mix(c, d, t)
    h = mix(e, f, t)
    i = mix(f, g, t)
    return mix(h, i, t)


def bezier_patch(p, u, v):
    p0 = cubic_bezier(p[0], p[4], p[8], p[12], v)
    p1 = cubic_bezier(p[1], p[5], p[9], p[13], v)
    p2 = cubic_bezier(p[2], p[6], p[10], p[14], v)
    p3 = cubic_bezier(p[3], p[7], p[11], p[15], v)
    return cubic_bezier(p0, p1, p2, p3, u)


# evaluate shape matrix for 3d beam element
def S_beam(u, v, l):
    s = np.array(np.zeros(6))
    s[0] = 1 - 3 * u**3 + 2 * u**3
    s[1] = l * (u - 2 * u**2 + u**3)
    s[2] = l * v * (1 - u)
    s[3] = 3 * u**2 - 2 * u**3
    s[4] = l * (-u**2 + u**3)
    s[5] = l * u * v
    return s


# e is a vector of time dependent absolute nodal coordinates
def product_Se(s, e):
    r = np.zeros([3, 1])
    for i in range(3):
        for j in range(len(s)):
            r[i] += s[j] * e[3 * j + i]


if __name__ == "__main__":

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    length = 2.0
    e = np.matrix(np.zeros([3, 6]))
    e[:, 0] = np.matrix([[0.0], [0.0], [0.0]])
    e[:, 1] = np.matrix([[2.0], [0.0], [0.0]])
    e[:, 2] = np.matrix([[0.0], [1.0], [0.0]])
    e[:, 3] = np.matrix([[2.0], [0.2], [0.0]])
    e[:, 4] = np.matrix([[2.0], [0.0], [0.0]])
    e[:, 5] = np.matrix([[0.0], [1.0], [0.0]])
    print(e)

    p0 = e[:, 0]
    pu0 = e[:, 1]
    p1 = e[:, 3]
    pu1 = e[:, 4]

    b0 = p0
    b1 = p0 + pu0 / 3.0
    b2 = p1 - pu1 / 3.0
    b3 = p1

    interp = np.zeros([3, 17])
    for k, t in enumerate(
            np.arange(0.0, 1.0 + 1.0 / 16.0,
                      1.0 / 16.0)):
        interp[:, k] = cubic_bezier(b0, b1, b2, b3, t).ravel()
    ax.plot(interp[0, :], interp[1, :], interp[2, :], c='b')
    ax.axis('equal')
    plt.show()

    # evaluate mass m

    # p00 = np
    #     # draw bezier splines
    #     for i in range(sections):
    #         for j in range(3):
    #             p0 = bezier_control_points[:, 30 * i + j * 3 + 0]
    #             p1 = bezier_control_points[:, 30 * i + j * 3 + 1]
    #             p2 = bezier_control_points[:, 30 * i + j * 3 + 2]
    #             p3 = bezier_control_points[:, 30 * i + j * 3 + 3]
    #             interp = np.zeros([3, 17])
    #             for k, t in enumerate(
    #                     np.arange(0.0, 1.0 + 1.0 / 16.0,
    #                               1.0 / 16.0)):
    #                 interp[:, k] = cubic_bezier(p0, p1, p2, p3, t).ravel()
    #             ax.plot(interp[0, :], interp[1, :], interp[2, :], c='b')
    #         for j in range(4):
    #             p0 = bezier_control_points[:, 30 * i + j * 3 + 0]
    #             p1 = bezier_control_points[:, 30 * i + 10 + j * 3 + 0]
    #             p2 = bezier_control_points[:, 30 * i + 20 + j * 3 + 0]
    #             if i < sections - 1:
    #                 p3 = bezier_control_points[:, 30 * i + 30 + j * 3 + 0]
    #             else:
    #                 p3 = bezier_control_points[:, j * 3 + 0]
    #
    #             interp = np.zeros([3, 17])
    #             for k, t in enumerate(
    #                     np.arange(0.0, 1.0 + 1.0 / 16.0,
    #                               1.0 / 16.0)):
    #                 interp[:, k] = cubic_bezier(p0, p1, p2, p3, t).ravel()
    #             ax.plot(interp[0, :], interp[1, :], interp[2, :], c='b')
