#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("usage: {} <input_file.csv>".format(sys.argv[0]))
        sys.exit(1)

    data = np.genfromtxt(sys.argv[1], names=True, delimiter=',', skip_footer=1)

    t = data["time"]

    ax1 = plt.subplot(411)
    for name in data.dtype.names[1:]:
        if "_q_" in name:
            plt.plot(t, data[name], label=name);
    plt.legend()
    ax1.set_ylabel('q (m/rad)')
    plt.ylim(-10, 10)

    ax2 = plt.subplot(412, sharex=ax1)
    for name in data.dtype.names[1:]:
        if "_dq_" in name:
            plt.plot(t, data[name], label=name);
    plt.ylim(-10, 10)
    ax2.set_ylabel('dq (m/s, rad/s)')
    plt.legend()

    ax3 = plt.subplot(413, sharex=ax2)
    for name in data.dtype.names[1:]:
        if "_ddq_" in name:
            plt.plot(t, data[name], label=name);
    plt.ylim(-20, 20)
    ax3.set_ylabel('ddq (m/s^2, rad/s^2)')
    plt.legend()

    ax4 = plt.subplot(414, sharex=ax3)
    plt.plot(t, data["constraint_error"], label="Constraint error");
    plt.legend()
    ax4.set_ylabel('sum of constraint error')
    ax4.set_xlabel('time (s)')

    fig = plt.gcf()
    fig.set_size_inches(9, 12)
    plt.savefig('states.png')
