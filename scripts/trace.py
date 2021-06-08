#!/usr/bin/env python

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import sys

if __name__ == "__main__":

    filepath = sorted([dir for dir in (Path.home() / '.shift').iterdir()])[-1]
    print(f"Found {filepath}")
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
                    if state[1].strip():
                        values = [float(i) for i in state[1].strip().split(" ")]
                        if label not in data:
                            data[label] = []
                        data[label].append(values)

    for key in data.keys():
        data[key] = np.array(data[key])

    folder = Path(filepath.stem)
    folder.mkdir(exist_ok=True)

    len_12_keys = ["d"]
    len_6_keys = ["h", "v", "aq", "p_", "g_"]
    len_2_keys = ["dc"]

    for key in data.keys():
        plt.clf()
        if key != "t" and key != "Ml_size_":
            print(f"Plotting '{key}'")
            mod_val = None
            labels = None

            if key in len_12_keys:
                mod_val = 12
                labels = ['m00', 'm10', 'm20',
                          'm01', 'm11', 'm21',
                          'm02', 'm12', 'm22',
                          'm03', 'm13', 'm23']
            elif key in len_6_keys:
                mod_val = 6
                labels = ['lin.x', 'lin.y', 'lin.z', 'ang.x', 'ang.y', 'ang.z']
            elif key in len_2_keys:
                mod_val = 2
                labels = ['s', 'c']

            if mod_val:
                num_subplots = int(data[key].shape[1] / mod_val)
                for i in range(num_subplots):
                    ax = plt.subplot(num_subplots, 1, i + 1)
                    for j in range(mod_val):
                         ax.plot(data["t"][:, 0], data[key][:, i * mod_val + j],
                              label=labels[j])
                    ax.set_ylabel(key + "_" + str(i))
                    ax.set_xlabel("time (s)")
                ax.legend()
                plt.gcf().suptitle(f"{filepath}: {key}")
            else:
                ax = plt.subplot(111)
                for i in range(data[key].shape[1]):
                    ax.plot(data["t"][:, 0], data[key][:, i],
                            label=key + "_" + str(i))
                    ax.set_xlabel("time (s)")
                    ax.set_title(key)

            plt.gcf().set_size_inches(9, 7.75)
            plt.savefig(folder / (key + '.png'))

            if labels:
                header = 't,'
                for i in range(int(data[key].shape[1] / mod_val)):
                    for j in range(mod_val):
                        header += key + "-" + str(i) + "-" + labels[j] + ","
            else:
                header = 't,'
                col = data[key].shape[1]
                if col > 0:
                    for i in range(col):
                        header += key + "-" + str(i) + ","
                else:
                    header += key + ","

            np.savetxt(
                folder / (key + '.csv'),
                np.concatenate((data["t"], data[key]), axis=1), fmt='%.6f',
                delimiter=',', newline='\n', header=header, footer='', comments='',
                encoding=None)

            if key in sys.argv[1:]:
                plt.show()

    # if we found integral error, e and de keys overlay them in one plot
    labels = ['lin.x', 'lin.y', 'lin.z', 'ang.x', 'ang.y', 'ang.z']
    plt.clf()
    num_loops = data["e"].shape[1] // 6
    for i in range(num_loops):
        plt.clf()
        for j in range(6):
            ax = plt.subplot(6, 1, j + 1)
            for val in ["integral_error", "e", "de"]:
                ax.set_ylabel(f"chord loop {i}\n{labels[j]}")
                ax.plot(data["t"][:, 0], data[val][:, i * 6 + j],
                        label=val)
            # ax.set_yscale("log")
        ax.legend()
        ax.set_xlabel("time (s)")
        plt.gcf().set_size_inches(9, 7.75)
        plt.suptitle(f"Constraint error for loop {i} in PO1")
        plt.savefig(folder / f"constraint_error_{i}.png")
        if "show_constraints" in sys.argv[1:]:
            plt.show()
