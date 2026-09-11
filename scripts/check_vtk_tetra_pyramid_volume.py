#!/usr/bin/env python3
"""Report fixed-orientation Tetra/Pyramid signed-volume failures in legacy VTK."""

import argparse
import itertools
import math


def signed_tetra(a, b, c, d):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    wx, wy, wz = (d[i] - a[i] for i in range(3))
    return (ux * (vy * wz - vz * wy)
            - uy * (vx * wz - vz * wx)
            + uz * (vx * wy - vy * wx)) / 6.0


def pyramid_integrated_jacobian(points):
    offset = 0.5 / math.sqrt(3.0)
    gauss = (0.5 - offset, 0.5 + offset)
    volume = 0.0
    for r, s, t in itertools.product(gauss, repeat=3):
        dr = tuple((1.0 - t) * (
            (1.0 - s) * (points[1][j] - points[0][j]) +
            s * (points[2][j] - points[3][j])) for j in range(3))
        ds = tuple((1.0 - t) * (
            (1.0 - r) * (points[3][j] - points[0][j]) +
            r * (points[2][j] - points[1][j])) for j in range(3))
        base = tuple(
            (1.0-r)*(1.0-s)*points[0][j] +
            r*(1.0-s)*points[1][j] + r*s*points[2][j] +
            (1.0-r)*s*points[3][j] for j in range(3))
        dt = tuple(points[4][j] - base[j] for j in range(3))
        volume += signed_tetra((0.0, 0.0, 0.0), dr, ds, dt) * 6.0 / 8.0
    return volume


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("vtk")
    args = parser.parse_args()

    with open(args.vtk, "r", encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            if fields and fields[0] == "POINTS":
                point_count = int(fields[1])
                break
        else:
            raise RuntimeError("POINTS section not found")

        points = []
        while len(points) < point_count:
            values = [float(value) for value in stream.readline().split()]
            points.extend(tuple(values[i:i + 3])
                          for i in range(0, len(values), 3))

        fields = stream.readline().split()
        while not fields or fields[0] != "CELLS":
            fields = stream.readline().split()
        cell_count = int(fields[1])
        candidate_cells = {}
        for cell_id in range(cell_count):
            fields = [int(value) for value in stream.readline().split()]
            if fields[0] in (4, 5):
                candidate_cells[cell_id] = fields[1:]

        fields = stream.readline().split()
        while not fields or fields[0] != "CELL_TYPES":
            fields = stream.readline().split()
        type_count = int(fields[1])
        types = []
        while len(types) < type_count:
            types.extend(int(value) for value in stream.readline().split())

    counts = {10: 0, 14: 0}
    failures = []
    minimum = {10: math.inf, 14: math.inf}
    minimum_integrated_pyramid = math.inf
    for cell_id, cell_type in enumerate(types):
        if cell_type not in counts:
            continue
        ids = candidate_cells[cell_id]
        counts[cell_type] += 1
        if cell_type == 10:
            subtets = ((0, 1, 2, 3),)
        else:
            subtets = ((0, 1, 2, 4), (0, 2, 3, 4))
        volumes = [signed_tetra(*(points[ids[i]] for i in subtet))
                   for subtet in subtets]
        integrated = (pyramid_integrated_jacobian(
            [points[index] for index in ids])
            if cell_type == 14 else volumes[0])
        minimum[cell_type] = min(minimum[cell_type], *volumes)
        if cell_type == 14:
            minimum_integrated_pyramid = min(
                minimum_integrated_pyramid, integrated)
        if (not all(math.isfinite(value) and value > 0.0 for value in volumes)
                or not math.isfinite(integrated) or integrated <= 0.0):
            failures.append((cell_id, cell_type, ids, volumes, integrated))

    print(f"tetra={counts[10]} pyramid={counts[14]}")
    print(f"minimum_tetra_subvolume={minimum[10]:.17g}")
    print(f"minimum_pyramid_subvolume={minimum[14]:.17g}")
    print(f"minimum_pyramid_integrated_volume="
          f"{minimum_integrated_pyramid:.17g}")
    print(f"nonpositive_or_nonfinite={len(failures)}")
    for failure in failures[:50]:
        print(failure)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
