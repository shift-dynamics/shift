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

    ax1 = plt.subplot(511)
    for name in data.dtype.names[1:]:
        if "_q_" in name:
            plt.plot(t, data[name], label=name)
    plt.legend(loc='upper left', bbox_to_anchor=(1.05, 1))
    ax1.set_ylabel('q (m,rad)')
    plt.ylim(-10, 10)

    ax2 = plt.subplot(512, sharex=ax1)
    for name in data.dtype.names[1:]:
        if "_dq_" in name:
            plt.plot(t, data[name], label=name)
    plt.ylim(-10, 10)
    ax2.set_ylabel('dq (m/s, rad/s)')

    ax3 = plt.subplot(513, sharex=ax2)
    for name in data.dtype.names[1:]:
        if "_ddq_" in name:
            plt.plot(t, data[name], label=name)
    plt.ylim(-20, 20)
    ax3.set_ylabel('ddq (m/s^2, rad/s^2)')

    ax4 = plt.subplot(514, sharex=ax3)
    plt.plot(t, data["constraint_error"], label="Constraint error")
    ax4.set_yscale('log')
    ax4.set_ylabel('Sum of constraint error')
    ax4.set_xlabel('time (s)')

    ax5 = plt.subplot(515, sharex=ax4)
    plt.plot(t, data["kinetic_energy"], label="Kinetic energy")
    plt.plot(t, data["gravitational_potential_energy"],
             label="Gravitational potential\nenergy")
    plt.plot(t, [i - data["total_energy"][0] for i in data["total_energy"]],
             label="Change in total\nenergy energy")
    ax5.set_ylabel('Energy (J)')
    ax5.set_xlabel('time (s)')
    ax5.legend()

    fig = plt.gcf()
    fig.set_size_inches(9, 7.75)
    plt.savefig('states.png', bbox_inches="tight", pad_inches=0.1)
    plt.show()
