#pragma once

#include <filesystem>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/io/cgns_surface_error.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    using CgnsSurfaceResult =
        Result<SurfaceMesh, CgnsSurfaceError>; // CGNS 表面读取结果

    CgnsSurfaceResult readCgnsSurface(
        const std::filesystem::path &cgns_path);
}
