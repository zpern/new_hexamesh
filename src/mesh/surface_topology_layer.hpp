#pragma once

#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh::detail
{
    enum class SurfaceTopologyLayer
    {
        NonInternal,
        Internal
    };

    constexpr SurfaceTopologyLayer topologyLayer(
        const SurfaceBoundaryTag &tag) noexcept
    {
        return tag.kind == SurfaceBoundaryKind::Internal
                   ? SurfaceTopologyLayer::Internal
                   : SurfaceTopologyLayer::NonInternal;
    }

    constexpr bool isInternalTopologyLayer(
        const SurfaceBoundaryTag &tag) noexcept
    {
        return topologyLayer(tag) == SurfaceTopologyLayer::Internal;
    }
}
