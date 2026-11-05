#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt, mpld3  # noqa
from matplotlib.ticker import FormatStrFormatter


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("usage: {} <input_file.csv>".format(sys.argv[0]))
        sys.exit(1)

    data = np.genfromtxt(sys.argv[1], names=True, delimiter=',', skip_footer=1)

    t = data["time"]

    total_plots = len(data.dtype.names) - 2
    fig, axs = plt.subplots(total_plots, 1, sharex='col')
    i = 0
    for name in data.dtype.names:
        if name == "time" or name == "f0":
            continue
        axs[i].set_title(name)
        axs[i].plot(t, data[name], label=name)
        axs[i].yaxis.set_major_formatter(FormatStrFormatter("%g"))
        axs[i].grid(visible=True, linestyle=":", color="grey")
        i += 1
    fig.set_size_inches(10, 2 * total_plots)
    fig.tight_layout()
    # plt.show()
    mpld3.show()

    # mpld3.save_html(fig, "excavator_plots.html")

