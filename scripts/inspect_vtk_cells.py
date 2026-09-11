#!/usr/bin/env python3
"""Inspect connectivity and ParaView quality values for selected VTK cells."""

import sys

from vtkmodules.vtkCommonDataModel import (
    VTK_HEXAHEDRON, VTK_PYRAMID, VTK_TETRA, VTK_WEDGE)
from vtkmodules.vtkFiltersVerdict import vtkMeshQuality
from vtkmodules.vtkIOLegacy import vtkUnstructuredGridReader


def quality_output(reader, setter):
    quality = vtkMeshQuality()
    quality.SetInputConnection(reader.GetOutputPort())
    getattr(quality, setter)()
    quality.Update()
    return quality.GetOutput().GetCellData().GetArray("Quality")


reader = vtkUnstructuredGridReader()
reader.SetFileName(sys.argv[1])
reader.ReadAllScalarsOn()
reader.Update()
grid = reader.GetOutput()

SETTERS = {
    VTK_TETRA: ("SetTetQualityMeasureToVolume", "SetTetQualityMeasureToScaledJacobian"),
    VTK_HEXAHEDRON: ("SetHexQualityMeasureToVolume", "SetHexQualityMeasureToScaledJacobian"),
    VTK_WEDGE: ("SetWedgeQualityMeasureToVolume", "SetWedgeQualityMeasureToScaledJacobian"),
    VTK_PYRAMID: ("SetPyramidQualityMeasureToVolume", "SetPyramidQualityMeasureToScaledJacobian"),
}


def subtract(a, b):
    return tuple(a[index] - b[index] for index in range(3))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def dot(a, b):
    return sum(a[index] * b[index] for index in range(3))

for argument in sys.argv[2:]:
    cell_id = int(argument)
    cell = grid.GetCell(cell_id)
    cell_type = grid.GetCellType(cell_id)
    point_ids = [cell.GetPointId(index) for index in range(cell.GetNumberOfPoints())]
    points = [grid.GetPoint(point_id) for point_id in point_ids]
    print(f"cell_id={cell_id} vtk_type={cell_type}")
    print(f"point_ids={point_ids}")
    for local_id, (point_id, point) in enumerate(zip(point_ids, points)):
        print(f"  local={local_id} point_id={point_id} xyz={point}")
    for field in ("source_face_id", "layer", "cell_role"):
        array = grid.GetCellData().GetArray(field)
        print(f"{field}={array.GetValue(cell_id) if array else 'unavailable'}")
    volume = quality_output(reader, SETTERS[cell_type][0])
    scaled = quality_output(reader, SETTERS[cell_type][1])
    print(f"paraview_volume={volume.GetValue(cell_id):.17g}")
    print(f"paraview_scaled_jacobian={scaled.GetValue(cell_id):.17g}")
    if cell_type == VTK_PYRAMID:
        base_center = tuple(sum(point[index] for point in points[:4]) / 4.0 for index in range(3))
        base_normal = cross(subtract(points[1], points[0]), subtract(points[3], points[0]))
        print(f"base_normal_dot_toward_apex={dot(base_normal, subtract(points[4], base_center)):.17g}")
