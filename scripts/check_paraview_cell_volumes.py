#!/usr/bin/env python3
"""Report ParaView/Verdict volume signs by 3-D VTK cell type."""

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


reader = vtkUnstructuredGridReader()
reader.SetFileName(sys.argv[1])
reader.ReadAllScalarsOn()
reader.Update()

quality = vtkMeshQuality()
quality.SetInputConnection(reader.GetOutputPort())
quality.SetTetQualityMeasureToVolume()
quality.SetHexQualityMeasureToVolume()
quality.SetWedgeQualityMeasureToVolume()
quality.SetPyramidQualityMeasureToVolume()
quality.Update()

grid = quality.GetOutput()
values = grid.GetCellData().GetArray("Quality")
for cell_type, name in TYPES.items():
    typed = [
        (cell_id, values.GetValue(cell_id))
        for cell_id in range(grid.GetNumberOfCells())
        if grid.GetCellType(cell_id) == cell_type
    ]
    negative = [item for item in typed if item[1] < 0.0]
    print(
        f"{name}: count={len(typed)} negative={len(negative)} "
        f"zero={sum(value == 0.0 for _, value in typed)} "
        f"positive={sum(value > 0.0 for _, value in typed)} "
        f"minimum={min((value for _, value in typed), default='n/a')}")
    for cell_id, value in negative[:20]:
        fields = []
        for field in ("source_face_id", "layer", "cell_role"):
            array = grid.GetCellData().GetArray(field)
            fields.append(
                f"{field}={int(array.GetValue(cell_id))}"
                if array is not None else f"{field}=unavailable")
        print(
            f"  cell_id={cell_id} volume={value:.17g} " +
            " ".join(fields))
