#!/usr/bin/env python3

import re
import csv
import matplotlib.pyplot as plt
import numpy as np


def plot_error():
    error_history = []
    targets = []
    elapsed_times = []
    with open("/home/jgleyze/cpcc2_evaluation/error_history_lin.txt") as f:
        for line in f.readlines():
            target_match = re.search(r"Target: \[([\d\.-]+), ([\d\.-]+), ([\d\.-]+)\]", line)
            error_match = re.search(r"Error: ([\d\.]+)", line)
            time_match = re.search(r"Elapsed Time: ([\d\.]+)", line)

            target_x = float(target_match.group(1))
            target_y = float(target_match.group(2))
            target_z = float(target_match.group(3))

            # Extraire l'erreur et le temps écoulé
            error = float(error_match.group(1))
            elapsed_time = float(time_match.group(1))

            targets.append((target_x, target_y, target_z))
            error_history.append(error)
            elapsed_times.append(elapsed_time)

    plt.figure()
    plt.scatter([str(i) for i in targets], error_history)
    for i, t in enumerate(targets):
        plt.annotate(f"{elapsed_times[i]:.2f}", (str(targets[i]), error_history[i]))


def parse_log_file():
    with open("/home/jgleyze/cpcc2_evaluation/full_history_lin.csv") as f:
        reader = csv.reader(f)
        rows_nb = sum(1 for row in reader)
        f.seek(0)
        times = []
        targets = np.empty((rows_nb, 3))
        errors = np.empty((rows_nb, 3))
        torques = np.empty((rows_nb, 5))

        for i, row in zip(range(rows_nb), reader):
            times.append(float(row[0]))
            targets[i] = [float(x) for x in row[1:4]]
            errors[i] = [float(x) for x in row[4:7]]
            torques[i] = [float(x) for x in row[7::]]

    return times, targets, errors, torques


def main():
    plot_error()
    times, targets, errors, torques = parse_log_file()
    plt.figure()
    plt.plot(times, [torques[i, :] for i in range(len(errors))])
    plt.show()


if __name__ == "__main__":
    main()
