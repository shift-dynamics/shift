#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as inter

if __name__ == "__main__":

    if len(sys.argv) != 3:
        print("usage: {} <input_file_1.csv> <input_file_2.csv>".format(
            sys.argv[0]))
        sys.exit(1)

    data1 = np.genfromtxt(sys.argv[1], names=True, delimiter=',', skip_footer=1)
    data2 = np.genfromtxt(sys.argv[2], names=True, delimiter=',', skip_footer=1)

    t_max = min(np.max(data1["time"]), np.max(data2["time"]))
    t = np.arange(0, t_max, 0.01)

    # find index where time stops being strictly increasing
    old_value = -1
    index = 0
    for value in data1["time"]:
        if value <= old_value:
            break
        old_value = value
        index = index + 1

    ax1 = plt.subplot(411)
    for name in data1.dtype.names[1:]:
        if "__q_" in name:
            print(data1["time"])
            data1_interp = \
                inter.InterpolatedUnivariateSpline(data1["time"][:index], data1[name][:index])
            data2_interp = \
                inter.InterpolatedUnivariateSpline(data2["time"][:index], data2[name][:index])
            y1 = data1_interp(t)
            y2 = data2_interp(t)
            diff = np.abs(y2 - y1)
            plt.plot(t, diff, label=name)
    plt.legend()
    ax1.set_ylabel('q (m/rad)')
    plt.ylim(-10, 10)


    ax2 = plt.subplot(412, sharex=ax1)
    for name in data1.dtype.names[1:]:
        if "__dq_" in name:
            data1_interp = \
                inter.InterpolatedUnivariateSpline(data1["time"][:index], data1[name][:index])
            data2_interp = \
                inter.InterpolatedUnivariateSpline(data2["time"][:index], data2[name][:index])
            y1 = data1_interp(t)
            y2 = data2_interp(t)
            diff = np.abs(y2 - y1)
            plt.plot(t, diff, label=name)
    plt.ylim(-10, 10)
    ax2.set_ylabel('dq (m/s, rad/s)')
    plt.legend()

    ax3 = plt.subplot(413, sharex=ax2)
    for name in data1.dtype.names[1:]:
        if "__ddq_" in name:
            data1_interp = \
                inter.InterpolatedUnivariateSpline(data1["time"][:index], data1[name][:index])
            data2_interp = \
                inter.InterpolatedUnivariateSpline(data2["time"][:index], data2[name][:index])
            y1 = data1_interp(t)
            y2 = data2_interp(t)
            diff = np.abs(y2 - y1)
            plt.plot(t, diff, label=name)
    plt.ylim(-20, 20)
    ax3.set_ylabel('ddq (m/s^2, rad/s^2)')
    plt.legend()

    ax4 = plt.subplot(414, sharex=ax3)
    ax4.set_ylabel('sum of constraint error')
    ax4.set_xlabel('time (s)')
    data1_interp = \
        inter.InterpolatedUnivariateSpline(
            data1["time"][:index], data1["constraint_error"][:index])
    data2_interp = \
        inter.InterpolatedUnivariateSpline(
            data2["time"][:index], data2["constraint_error"][:index])
    y1 = data1_interp(t)
    y2 = data2_interp(t)
    diff = np.abs(y2 - y1)
    plt.plot(t, diff, label="constraint_error")
    plt.legend()

    fig = plt.gcf()
    fig.set_size_inches(9, 12)
    plt.savefig('states_diff.png')
