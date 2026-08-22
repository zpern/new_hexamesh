#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>
#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/growth/regular_layer_growth_error.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology.hpp>

namespace boundary_mesh
{
    class RegularLayerGenerator
    {
    public:
        /// 从第 0 层 Wall Front 生成独立的规则边界层体网格。
        Result<RegularLayerGrowthResult, RegularLayerGrowthError>
        generate(
            const SurfaceMesh &surface_mesh,
            const SurfaceTopology &topology,
            const GrowthPatch &patch,
            const GrowthFront &initial_front,
            const std::vector<SourceVertexGrowthProfile> &profiles,
            const RegularLayerGrowthOptions &options = {}) const;
    };

    Result<RegularLayerGrowthResult, RegularLayerGrowthError>
    generateRegularLayers(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options = {});
}
