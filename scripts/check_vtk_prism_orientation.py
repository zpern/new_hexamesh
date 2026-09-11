#!/usr/bin/env python3
"""Compare fixed Prism orientation with the opposite triangular winding."""

import argparse


def signed_tetra(a, b, c, d):
    u = tuple(b[i] - a[i] for i in range(3))
    v = tuple(c[i] - a[i] for i in range(3))
    w = tuple(d[i] - a[i] for i in range(3))
    return (u[0] * (v[1] * w[2] - v[2] * w[1])
            - u[1] * (v[0] * w[2] - v[2] * w[0])
            + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("vtk")
    args = parser.parse_args()
    lines = open(args.vtk, encoding="ascii").read().splitlines()
    ph = next(i for i, line in enumerate(lines) if line.startswith("POINTS "))
    point_count = int(lines[ph].split()[1])
    points = [tuple(map(float, lines[ph + 1 + i].split()))
              for i in range(point_count)]
    ch = next(i for i, line in enumerate(lines) if line.startswith("CELLS "))
    cell_count = int(lines[ch].split()[1])
    cells = [list(map(int, lines[ch + 1 + i].split()))[1:]
             for i in range(cell_count)]
    th = next(i for i, line in enumerate(lines)
              if line.startswith("CELL_TYPES "))
    types = [int(lines[th + 1 + i]) for i in range(cell_count)]
    values = []
    for cell_id, cell_type in enumerate(types):
        if cell_type != 13:
            continue
        p = [points[index] for index in cells[cell_id]]
        subtets = ((0, 1, 2, 3), (1, 2, 3, 4), (2, 3, 4, 5))
        volume = sum(signed_tetra(*(p[index] for index in tet))
                     for tet in subtets)
        opposite = -volume
        values.append((cell_id, volume, opposite))
    print(f"prisms={len(values)}")
    print(f"internal_negative={sum(value[1] < 0 for value in values)}")
    print(f"opposite_winding_negative={sum(value[2] < 0 for value in values)}")
    print("first=", values[:5])


if __name__ == "__main__":
    main()
