#!/usr/bin/env python3
import collections
import sys


CELL_FACES = {
    10: ((0, 1, 2), (0, 3, 1), (1, 3, 2), (2, 3, 0)),
    12: ((0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
         (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)),
    13: ((0, 1, 2), (3, 5, 4), (0, 3, 4, 1),
         (1, 4, 5, 2), (2, 5, 3, 0)),
    14: ((0, 1, 2, 3), (0, 4, 1), (1, 4, 2),
         (2, 4, 3), (3, 4, 0)),
}


def read_cells(path):
    points = []
    cells = []
    types = []
    with open(path, "r", encoding="ascii") as stream:
        lines = iter(stream)
        for line in lines:
            if line.startswith("POINTS "):
                count = int(line.split()[1])
                points = [tuple(float(value) for value in next(lines).split())
                          for _ in range(count)]
            elif line.startswith("CELLS "):
                count = int(line.split()[1])
                for _ in range(count):
                    fields = [int(value) for value in next(lines).split()]
                    cells.append(tuple(fields[1:]))
            elif line.startswith("CELL_TYPES "):
                count = int(line.split()[1])
                types = [int(next(lines)) for _ in range(count)]
                break
    if len(cells) != len(types):
        raise RuntimeError(f"cell/type mismatch: {len(cells)} != {len(types)}")
    return points, cells, types


def main():
    points, cells, types = read_cells(sys.argv[1])
    if types and all(cell_type in (5, 9) for cell_type in types):
        unique = set()
        duplicates = 0
        counts = collections.Counter()
        for cell, cell_type in zip(cells, types):
            key = tuple(sorted(points[index] for index in cell))
            duplicates += key in unique
            unique.add(key)
            counts[cell_type] += 1
        print(f"cells={len(cells)}")
        print(f"duplicate_faces={duplicates}")
        print("cell_types=" + ",".join(
            f"{cell_type}:{count}"
            for cell_type, count in sorted(counts.items())))
        return 1 if duplicates else 0
    owners = collections.defaultdict(int)
    canonical_cells = set()
    duplicates = 0
    for cell, cell_type in zip(cells, types):
        key = (cell_type, tuple(sorted(cell)))
        if key in canonical_cells:
            duplicates += 1
        canonical_cells.add(key)
        for local_face in CELL_FACES[cell_type]:
            face = tuple(sorted(cell[index] for index in local_face))
            owners[face] += 1
    histogram = collections.Counter(owners.values())
    boundary = collections.Counter(len(face) for face, count in owners.items()
                                   if count == 1)
    print(f"cells={len(cells)}")
    print(f"duplicate_cells={duplicates}")
    print("owner_histogram=" + ",".join(
        f"{owners_count}:{face_count}"
        for owners_count, face_count in sorted(histogram.items())))
    print("boundary_faces=" + ",".join(
        f"{vertices}:{face_count}"
        for vertices, face_count in sorted(boundary.items())))
    if len(sys.argv) > 2:
        surface_points, surface_cells, surface_types = read_cells(sys.argv[2])
        volume_boundary = {
            tuple(sorted(points[index] for index in face))
            for face, count in owners.items() if count == 1
        }
        surface_faces = {
            tuple(sorted(surface_points[index] for index in cell))
            for cell, cell_type in zip(surface_cells, surface_types)
            if cell_type == 5
        }
        print(f"surface_unique_triangles={len(surface_faces)}")
        print(f"surface_not_volume_boundary="
              f"{len(surface_faces - volume_boundary)}")
        print(f"matched_surface_triangles="
              f"{len(surface_faces & volume_boundary)}")
    return 1 if duplicates or any(count > 2 for count in histogram) else 0


if __name__ == "__main__":
    raise SystemExit(main())
