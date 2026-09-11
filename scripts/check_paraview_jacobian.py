#!/usr/bin/env python3
"""Report ParaView/Verdict Jacobian signs by 3-D cell type."""

import sys

from vtkmodules.vtkCommonDataModel import (
    VTK_HEXAHEDRON, VTK_PYRAMID, VTK_TETRA, VTK_WEDGE)
from vtkmodules.vtkFiltersVerdict import vtkMeshQuality
from vtkmodules.vtkIOLegacy import vtkUnstructuredGridReader

TYPES = {
    VTK_TETRA: ("Tet", "SetTetQualityMeasureToJacobian"),
    VTK_HEXAHEDRON: ("Hex", "SetHexQualityMeasureToJacobian"),
    VTK_WEDGE: ("Wedge", "SetWedgeQualityMeasureToJacobian"),
    VTK_PYRAMID: ("Pyramid", "SetPyramidQualityMeasureToJacobian"),
}

reader = vtkUnstructuredGridReader()
reader.SetFileName(sys.argv[1])
reader.ReadAllScalarsOn()
reader.Update()

for cell_type, (name, setter) in TYPES.items():
    quality = vtkMeshQuality()
    quality.SetInputConnection(reader.GetOutputPort())
    getattr(quality, setter)()
    quality.Update()
    grid = quality.GetOutput()
    values = grid.GetCellData().GetArray("Quality")
    typed = [
        (cell_id, values.GetValue(cell_id))
        for cell_id in range(grid.GetNumberOfCells())
        if grid.GetCellType(cell_id) == cell_type
    ]
    negative = [item for item in typed if item[1] < 0.0]
    print(f"{name}: count={len(typed)} negative={len(negative)} "
          f"zero={sum(value == 0.0 for _, value in typed)} "
          f"minimum={min((value for _, value in typed), default='n/a')}")
    for cell_id, value in negative:
        fields = []
        for field in ("source_face_id", "layer", "cell_role"):
            array = grid.GetCellData().GetArray(field)
            fields.append(f"{field}={int(array.GetValue(cell_id))}")
        print(f"  cell_id={cell_id} jacobian={value:.17g} " + " ".join(fields))
