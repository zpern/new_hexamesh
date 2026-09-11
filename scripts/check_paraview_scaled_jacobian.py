#!/usr/bin/env python3
"""Report ParaView/Verdict scaled-Jacobian signs by 3-D cell type."""

import sys

from vtkmodules.vtkCommonDataModel import (
    VTK_HEXAHEDRON,
    VTK_PYRAMID,
    VTK_TETRA,
    VTK_WEDGE,
)
from vtkmodules.vtkFiltersVerdict import vtkMeshQuality
from vtkmodules.vtkIOLegacy import vtkUnstructuredGridReader


TYPES = {
    VTK_TETRA: "Tet",
    VTK_HEXAHEDRON: "Hex",
    VTK_WEDGE: "Wedge",
    VTK_PYRAMID: "Pyramid",
}


def signed_tetra(a, b, c, d):
    u = tuple(b[index] - a[index] for index in range(3))
    v = tuple(c[index] - a[index] for index in range(3))
    w = tuple(d[index] - a[index] for index in range(3))
    return (
        u[0] * (v[1] * w[2] - v[2] * w[1]) -
        u[1] * (v[0] * w[2] - v[2] * w[0]) +
        u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0

reader = vtkUnstructuredGridReader()
reader.SetFileName(sys.argv[1])
reader.ReadAllScalarsOn()
reader.Update()

quality = vtkMeshQuality()
quality.SetInputConnection(reader.GetOutputPort())
quality.SetTetQualityMeasureToScaledJacobian()
quality.SetHexQualityMeasureToScaledJacobian()
quality.SetWedgeQualityMeasureToScaledJacobian()
quality.SetPyramidQualityMeasureToScaledJacobian()
quality.Update()

grid = quality.GetOutput()
values = grid.GetCellData().GetArray("Quality")
for cell_type, name in TYPES.items():
    typed = [
        (cell_id, values.GetValue(cell_id))
        for cell_id in range(grid.GetNumberOfCells())
        if grid.GetCellType(cell_id) == cell_type
    ]
    nonpositive = [item for item in typed if item[1] <= 0.0]
    print(
        f"{name}: count={len(typed)} "
        f"negative={sum(value < 0.0 for _, value in typed)} "
        f"zero={sum(value == 0.0 for _, value in typed)} "
        f"positive={sum(value > 0.0 for _, value in typed)} "
        f"minimum={min((value for _, value in typed), default='n/a')}")
    for cell_id, value in nonpositive:
        fields = []
        for field in ("source_face_id", "layer", "cell_role"):
            array = grid.GetCellData().GetArray(field)
            fields.append(
                f"{field}={int(array.GetValue(cell_id))}"
                if array is not None else f"{field}=unavailable")
        print(
            f"  cell_id={cell_id} scaled_jacobian={value:.17g} " +
            " ".join(fields))
        if cell_type == VTK_PYRAMID:
            cell = grid.GetCell(cell_id)
            points = [
                cell.GetPoints().GetPoint(index)
                for index in range(cell.GetNumberOfPoints())
            ]
            print(
                "    fixed_subtets=" +
                f"{signed_tetra(points[0], points[1], points[2], points[4]):.17g}," +
                f"{signed_tetra(points[0], points[2], points[3], points[4]):.17g}")
