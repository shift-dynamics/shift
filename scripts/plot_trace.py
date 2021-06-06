#!/usr/bin/env python

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import sys

if __name__ == "__main__":

    filepath = sorted([dir for dir in (Path.home() / '.shift').iterdir()])[-1]
    data = {}
    with filepath.open() as f:
        while True:
            line = f.readline()
            if not line:
                break
            stripped = line.strip()
            if stripped.startswith("[TRACE") and \
                    stripped.endswith("RNCA States"):
                while True:
                    state = f.readline().strip().split(":")
                    if not state[0].strip():
                        break
                    label = state[0].strip()
                    values = [float(i) for i in state[1].strip().split(" ")]
                    if label not in data:
                        data[label] = []
                    data[label].append(values)

    for key in data.keys():
        data[key] = np.array(data[key])

    folder = Path(filepath.stem)
    folder.mkdir(exist_ok=True)

    len_12_keys = ["d"]
    len_6_keys = ["h", "v", "aq", "p_"]
    len_2_keys = ["dc"]

    for key in data.keys():
        plt.clf()
        if key != "t" and key != "Ml_size_":
            print(f"Plotting {key}")
            mod_val = None
            if key in len_12_keys:
                mod_val = 12
            elif key in len_6_keys:
                mod_val = 6
            elif key in len_2_keys:
                mod_val = 2

            if mod_val:
                num_subplots = int(data[key].shape[1] / mod_val)
                for i in range(num_subplots):
                    ax = plt.subplot(num_subplots, 1, i + 1)
                    for j in range(mod_val):
                         ax.plot(data["t"][:, 0], data[key][:, i * mod_val + j],
                              label=key + "_" + str(i))
                    ax.set_xlabel("time (s)")
                plt.gcf().suptitle(key)
            else:
                ax = plt.subplot(111)
                for i in range(data[key].shape[1]):
                    ax.plot(data["t"][:, 0], data[key][:, i],
                            label=key + "_" + str(i))
                    ax.set_xlabel("time (s)")
                    ax.set_title(key)
                plt.gcf().set_size_inches(9, 7.75)
                plt.savefig(folder / (key + '.png'))
            if key in sys.argv[1:]:
                plt.show()
