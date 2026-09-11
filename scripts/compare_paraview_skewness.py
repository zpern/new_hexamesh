#!/usr/bin/env python3
"""Compare ParaView 6.1 equiangle-skew counts between legacy VTK meshes."""

import sys
from collections import Counter

from vtkmodules.vtkCommonDataModel import (
    VTK_HEXAHEDRON,
    VTK_PYRAMID,
    VTK_TETRA,
    VTK_WEDGE,
)
from vtkmodules.vtkFiltersVerdict import vtkMeshQuality
from vtkmodules.vtkIOLegacy import vtkUnstructuredGridReader


TYPE_NAMES = {
    VTK_TETRA: "Tet",
    VTK_HEXAHEDRON: "Hex",
    VTK_WEDGE: "Wedge",
    VTK_PYRAMID: "Pyramid",
}


def report(path):
    reader = vtkUnstructuredGridReader()
    reader.SetFileName(path)
    reader.ReadAllScalarsOn()
    reader.Update()

    quality = vtkMeshQuality()
    quality.SetInputConnection(reader.GetOutputPort())
    quality.SetTetQualityMeasureToEquiangleSkew()
    quality.SetHexQualityMeasureToEquiangleSkew()
    quality.SetWedgeQualityMeasureToEquiangleSkew()
    quality.SetPyramidQualityMeasureToEquiangleSkew()
    quality.Update()

    grid = quality.GetOutput()
    values = grid.GetCellData().GetArray("Quality")
    print(path)
    total_high = 0
    for cell_type, name in TYPE_NAMES.items():
        typed = [
            values.GetValue(cell_id)
            for cell_id in range(grid.GetNumberOfCells())
            if grid.GetCellType(cell_id) == cell_type
        ]
        high = sum(value > 0.9 for value in typed)
        total_high += high
        print(
            f"  {name}: count={len(typed)} skew_gt_0.9={high} "
            f"minimum={min(typed) if typed else 'n/a'} "
            f"maximum={max(typed) if typed else 'n/a'}")
        role_array = grid.GetCellData().GetArray("cell_role")
        layer_array = grid.GetCellData().GetArray("layer")
        high_details = Counter()
        if role_array is not None and layer_array is not None:
            for cell_id in range(grid.GetNumberOfCells()):
                if (grid.GetCellType(cell_id) == cell_type and
                        values.GetValue(cell_id) > 0.9):
                    high_details[(int(role_array.GetValue(cell_id)),
                                  int(layer_array.GetValue(cell_id)))] += 1
        print(f"    high_by_role_layer={dict(sorted(high_details.items()))}")
    print(f"  total_skew_gt_0.9={total_high}")


for filename in sys.argv[1:]:
    report(filename)
