#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import scipy.optimize as opt


def rot_y(degrees, points):
    c = np.cos(np.radians(degrees))
    s = np.sin(np.radians(degrees))
    R = np.matrix([
        [c, 0.0, -s],
        [0.0, 1.0, 0.0],
        [s, 0.0, c]
    ])
    return R * points


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


def find_k_error(k, degrees, p0):
    tangent = np.matrix([[1.0], [0.0], [0.0]])
    r = np.fabs(p0[2])
    p3 = rot_y(degrees, p0)
    p1 = p0 + tangent * k * r
    p2 = p3 - rot_y(degrees, tangent) * k * r
    p = cubic_bezier(p0, p1, p2, p3, 0.5)
    return np.fabs(r - np.sqrt(p[0] * p[0] + p[2] * p[2]))**2


def normalized(v):
    norm = np.linalg.norm(v, ord=1)
    if norm == 0:
        norm = np.finfo(v.dtype).eps
    return v / norm


def find_p_error(p, angle, p0):
    tangent = np.matrix([[1.0], [0.0], [0.0]])
    p1 = p0 + tangent * p
    u = normalized(p0)
    v = normalized(p1)
    u[1] = 0
    v[1] = 0
    u = normalized(u)
    v = normalized(v)
    return((np.arccos(u.transpose() * v) - angle)**2)


if __name__ == "__main__":

    sections = 12  # total number of revolved sections
    control_points = np.matrix(np.zeros([3, 3 * 10 * sections]))

    # control points for a cross section in yz plane
    control_points_base = np.matrix([
        [0.0, -71.0, -190.5],  # intersects this pt
        [0.0, -105, -224.5],
        [0.0, -105, -260.294],
        [0.0, -71.0, -292.0],  # intersects this pt
        [0.0, -35.0, -325.571],
        [0.0, 35.0, -325.571],
        [0.0, 71.0, -292.0],  # intersects this pt
        [0.0, 105, -260.294],
        [0.0, 105, -224.5],
        [0.0, 71.0, -190.5]  # intersects this pt
    ]).transpose()

    for i in range(3):
        p0 = np.matrix(control_points_base[:, i * 3 + 0])
        p1 = np.matrix(control_points_base[:, i * 3 + 1])
        p2 = np.matrix(control_points_base[:, i * 3 + 2])
        p3 = np.matrix(control_points_base[:, i * 3 + 3])

        tangent = np.matrix([[1.0], [0.0], [0.0]])

        # find a value of 'k' that approximates a circular profile
        k = opt.minimize(
                    find_k_error, 0.175539, args=(360.0 / sections,  p0)).x[0]
        r = np.fabs(p0[2])
        p12 = rot_y(360.0 / sections, p0)
        p4 = p0 + tangent * k * r
        p8 = p12 - rot_y(360.0 / sections, tangent) * k * r

        k = opt.minimize(
                    find_k_error, 0.175539, args=(360.0 / sections,  p3)).x[0]
        r = np.fabs(p3[2])
        tangent = np.matrix([[1.0], [0.0], [0.0]])
        p15 = rot_y(360.0 / sections, p3)
        p7 = p3 + tangent * k * r
        p11 = p15 - rot_y(360.0 / sections, tangent) * k * r

        # find angle between p0 and p4
        # cos \theta = (u \dot v) / (|u| * |v|)
        u = normalized(p0)
        v = normalized(p4)
        u[1] = 0
        v[1] = 0
        u = normalized(u)
        v = normalized(v)
        angle = np.arccos(u.transpose() * v)
        print('angle:', angle)
        print(360.0 / sections)

        k = opt.minimize(
                    find_p_error, 100, args=(angle,  p1)).x[0]
        print(k)
        p5 = p1 + tangent * k
        p13 = rot_y(360.0 / sections, p1)
        p9 = p13 - rot_y(360.0 / sections, tangent) * k

        k = opt.minimize(
                    find_p_error, 100, args=(angle,  p2)).x[0]
        p6 = p2 + tangent * k
        p14 = rot_y(360.0 / sections, p2)
        p10 = p14 - rot_y(360.0 / sections, tangent) * k

        control_points[:, 10 * 0 + 3 * i + 0] = p0
        control_points[:, 10 * 0 + 3 * i + 1] = p1
        control_points[:, 10 * 0 + 3 * i + 2] = p2

        control_points[:, 10 * 1 + 3 * i + 0] = p4
        control_points[:, 10 * 1 + 3 * i + 1] = p5
        control_points[:, 10 * 1 + 3 * i + 2] = p6

        control_points[:, 10 * 2 + 3 * i + 0] = p8
        control_points[:, 10 * 2 + 3 * i + 1] = p9
        control_points[:, 10 * 2 + 3 * i + 2] = p10

        if i == 2:
            control_points[:, 10 * 0 + 3 * i + 3] = p3
            control_points[:, 10 * 1 + 3 * i + 3] = p7
            control_points[:, 10 * 2 + 3 * i + 3] = p11

    # revolve control points
    for i in range(1, sections):
        control_points[:, 30 * i: 30 * i + 30] = \
            rot_y(i * 360.0 / sections, control_points[:, :30])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # draw bezier splines
    for i in range(sections):
        for j in range(3):
            p0 = control_points[:, 30 * i + j * 3 + 0]
            p1 = control_points[:, 30 * i + j * 3 + 1]
            p2 = control_points[:, 30 * i + j * 3 + 2]
            p3 = control_points[:, 30 * i + j * 3 + 3]
            interp = np.zeros([3, 17])
            for k, t in enumerate(
                    np.arange(0.0, 1.0 + 1.0 / 16.0,
                              1.0 / 16.0)):
                interp[:, k] = cubic_bezier(p0, p1, p2, p3, t).ravel()
            ax.plot(interp[0, :], interp[1, :], interp[2, :], c='b')
        for j in range(4):
            p0 = control_points[:, 30 * i + j * 3 + 0]
            p1 = control_points[:, 30 * i + 10 + j * 3 + 0]
            p2 = control_points[:, 30 * i + 20 + j * 3 + 0]
            if i < sections - 1:
                p3 = control_points[:, 30 * i + 30 + j * 3 + 0]
            else:
                p3 = control_points[:, j * 3 + 0]

            interp = np.zeros([3, 17])
            for k, t in enumerate(
                    np.arange(0.0, 1.0 + 1.0 / 16.0,
                              1.0 / 16.0)):
                interp[:, k] = cubic_bezier(p0, p1, p2, p3, t).ravel()
            ax.plot(interp[0, :], interp[1, :], interp[2, :], c='b')

    # draw bezier patches
    for i in range(sections):
        for j in range(3):
            p0 = control_points[:, 30 * i + j * 3 + 0]
            p1 = control_points[:, 30 * i + j * 3 + 1]
            p2 = control_points[:, 30 * i + j * 3 + 2]
            p3 = control_points[:, 30 * i + j * 3 + 3]

            p4 = control_points[:, 30 * i + 10 + j * 3 + 0]
            p5 = control_points[:, 30 * i + 10 + j * 3 + 1]
            p6 = control_points[:, 30 * i + 10 + j * 3 + 2]
            p7 = control_points[:, 30 * i + 10 + j * 3 + 3]

            p8 = control_points[:, 30 * i + 20 + j * 3 + 0]
            p9 = control_points[:, 30 * i + 20 + j * 3 + 1]
            p10 = control_points[:, 30 * i + 20 + j * 3 + 2]
            p11 = control_points[:, 30 * i + 20 + j * 3 + 3]

            if i < sections - 1:
                p12 = control_points[:, 30 * i + 30 + j * 3 + 0]
                p13 = control_points[:, 30 * i + 30 + j * 3 + 1]
                p14 = control_points[:, 30 * i + 30 + j * 3 + 2]
                p15 = control_points[:, 30 * i + 30 + j * 3 + 3]
            else:
                p12 = control_points[:, j * 3 + 0]
                p13 = control_points[:, j * 3 + 1]
                p14 = control_points[:, j * 3 + 2]
                p15 = control_points[:, j * 3 + 3]
            x = np.matrix(np.zeros([17, 17]))
            y = np.matrix(np.zeros([17, 17]))
            z = np.matrix(np.zeros([17, 17]))
            for u_i, u in enumerate(np.linspace(0, 1, 17)):
                for v_j, v in enumerate(np.linspace(0, 1, 17)):
                    p = bezier_patch(
                        [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10,
                         p11, p12, p13, p14, p15], u, v)
                    x[u_i, v_j] = p[0]
                    y[u_i, v_j] = p[1]
                    z[u_i, v_j] = p[2]
            ax.plot_surface(x, y, z, alpha=0.5)

    print(control_points.shape)

    # print control points
    ax.scatter(
        control_points[0, :],
        control_points[1, :],
        control_points[2, :], c='r', marker='o')

    ax.axis('equal')
    plt.show()
