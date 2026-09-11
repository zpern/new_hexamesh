#!/usr/bin/env python3
"""Report ParaView/VTK volume signs for VTK_WEDGE cells."""

import sys

from vtkmodules.vtkCommonDataModel import VTK_WEDGE
from vtkmodules.vtkFiltersVerdict import vtkMeshQuality
from vtkmodules.vtkIOLegacy import vtkUnstructuredGridReader


def wedge_values(reader, setter):
    quality = vtkMeshQuality()
    quality.SetInputConnection(reader.GetOutputPort())
    getattr(quality, setter)()
    quality.Update()
    grid = quality.GetOutput()
    values = grid.GetCellData().GetArray("Quality")
    return grid, values


def main():
    reader = vtkUnstructuredGridReader()
    reader.SetFileName(sys.argv[1])
    reader.ReadAllScalarsOn()
    reader.Update()

    grid, values = wedge_values(reader, "SetWedgeQualityMeasureToVolume")
    volumes = [
        values.GetValue(cell_id)
        for cell_id in range(grid.GetNumberOfCells())
        if grid.GetCellType(cell_id) == VTK_WEDGE
    ]
    print(f"wedge_count={len(volumes)}")
    print(f"negative={sum(value < 0.0 for value in volumes)}")
    print(f"zero={sum(value == 0.0 for value in volumes)}")
    print(f"positive={sum(value > 0.0 for value in volumes)}")
    if volumes:
        print(f"minimum={min(volumes):.17g}")
        print(f"maximum={max(volumes):.17g}")
        print("first=" + ",".join(f"{value:.17g}" for value in volumes[:5]))

    source_faces = grid.GetCellData().GetArray("source_face_id")
    for setter in (
        "SetWedgeQualityMeasureToVolume",
        "SetWedgeQualityMeasureToJacobian",
        "SetWedgeQualityMeasureToScaledJacobian",
    ):
        measured_grid, measured = wedge_values(reader, setter)
        matches = []
        for cell_id in range(measured_grid.GetNumberOfCells()):
            if (measured_grid.GetCellType(cell_id) == VTK_WEDGE and
                    int(source_faces.GetValue(cell_id)) == 25563):
                matches.append((cell_id, measured.GetValue(cell_id)))
        print(f"{setter}: {matches[:10]}")

    scaled_grid, scaled = wedge_values(
        reader, "SetWedgeQualityMeasureToScaledJacobian")
    scaled_wedges = [
        (cell_id, scaled.GetValue(cell_id))
        for cell_id in range(scaled_grid.GetNumberOfCells())
        if scaled_grid.GetCellType(cell_id) == VTK_WEDGE
    ]
    negative_scaled = [item for item in scaled_wedges if item[1] < 0.0]
    print(f"scaled_jacobian_negative={len(negative_scaled)}")
    for cell_id, value in negative_scaled[:20]:
        fields = []
        for name in ("source_face_id", "layer", "cell_role"):
            array = scaled_grid.GetCellData().GetArray(name)
            fields.append(
                f"{name}={int(array.GetValue(cell_id))}"
                if array is not None else f"{name}=unavailable")
        print(f"negative_scaled cell_id={cell_id} value={value:.17g} " +
              " ".join(fields))


if __name__ == "__main__":
    main()
