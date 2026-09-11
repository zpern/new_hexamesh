#!/usr/bin/env python3
"""Count exact and rounded-coordinate duplicates in an ASCII legacy VTK."""

import argparse
from collections import Counter


def inspect(path):
    with open(path, "r", encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if fields and fields[0] == "POINTS":
                count = int(fields[1])
                break
        points = []
        while len(points) < count:
            values = [float(value) for value in stream.readline().split()]
            points.extend(tuple(values[i:i + 3])
                          for i in range(0, len(values), 3))

        fields = stream.readline().split()
        while not fields or fields[0] != "CELLS":
            fields = stream.readline().split()
        cell_count = int(fields[1])
        used = set()
        for _ in range(cell_count):
            fields = [int(value) for value in stream.readline().split()]
            used.update(fields[1:])

    exact = Counter(points)
    rounded = Counter(tuple(round(value, 9) for value in point)
                      for point in points)
    print(path)
    print(f"points={len(points)} cells={cell_count} used_points={len(used)}")
    print(f"exact_unique={len(exact)} exact_duplicate_entries="
          f"{len(points)-len(exact)} max_multiplicity={max(exact.values())}")
    print(f"tol_1e-9_unique={len(rounded)} tol_1e-9_duplicate_entries="
          f"{len(points)-len(rounded)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("vtk", nargs="+")
    args = parser.parse_args()
    for path in args.vtk:
        inspect(path)


if __name__ == "__main__":
    main()
