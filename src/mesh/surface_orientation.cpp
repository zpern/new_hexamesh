#include <algorithm>
#include <variant>

#include <boundary_mesh/mesh/mesh_surface_orientation.hpp>

namespace boundary_mesh
{
    void reverseSurfaceOrientation(
        SurfaceMesh &mesh) noexcept
    {
        for (SurfaceFace &face : mesh.faces)
        {
            std::visit(
                [](auto &value)
                {
                    std::reverse(
                        value.vertex_ids.begin() + 1,
                        value.vertex_ids.end());
                },
                face);
        }
    }
}
