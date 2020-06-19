import csv
import statistics
from os.path import exists


def stats(f):
    if not exists(f):
        print("error in file input")
        return -1, -1

    with open(f) as input:
        samples = csv.reader(input, delimiter=',')

        next(samples)  # skip header
        values = []
        for row in samples:
            values.append(float(row[1]))

        return statistics.mean(values), statistics.stdev(values)


m1, std1 = stats("./config/log/Graph1.txt")
print(f"Graph1 mean: {m1:4.4f}, stdev: {std1:4.4f}")
m2, std2 = stats("./config/log/Graph2.txt")
print(f"Graph2 mean: {m2:4.4f}, stdev: {std2:4.4f}")
