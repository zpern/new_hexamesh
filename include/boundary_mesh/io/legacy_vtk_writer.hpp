#pragma once

#include <filesystem>
#include <variant>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/io/vtk_write_error.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_volume.hpp>

namespace boundary_mesh
{
    using VtkWriteStatus =
        Result<std::monostate, VtkWriteError>; // Legacy VTK 写出结果

    VtkWriteStatus writeLegacyVtk(
        const std::filesystem::path &path,
        const SurfaceMesh &mesh);

    VtkWriteStatus writeLegacyVtk(
        const std::filesystem::path &path,
        const VolumeMesh &mesh);
}
