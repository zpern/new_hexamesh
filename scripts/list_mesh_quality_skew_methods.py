from vtkmodules.vtkFiltersVerdict import vtkMeshQuality

quality = vtkMeshQuality()
for name in dir(quality):
    if "Skew" in name or "QualityMeasureTo" in name and "Equi" in name:
        print(name)
