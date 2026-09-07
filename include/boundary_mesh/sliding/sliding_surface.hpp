#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/spatial/triangle_surface_index.hpp>

namespace boundary_mesh
{
    enum class SlidingSurfaceKind { AxisX, AxisY, AxisZ, Curved };

    struct SlidingSurface
    {
        std::uint32_t region_id{};
        SurfaceBoundaryKind boundary_kind{SurfaceBoundaryKind::Symmetry};
        SlidingSurfaceKind kind{SlidingSurfaceKind::Curved};
        Scalar axis_value{};
        Scalar reference_length{};
        Scalar projection_tolerance{};
        std::optional<TriangleSurfaceIndex> curved_index;
    };

    class SlidingSurfaceBuilder;

    class SlidingSurfaceSet
    {
    public:
        const SlidingSurface *find(std::uint32_t region_id) const noexcept;
        const std::vector<SlidingSurface> &surfaces() const noexcept;

    private:
        friend class SlidingSurfaceBuilder;
        std::vector<SlidingSurface> surfaces_;
    };
}
