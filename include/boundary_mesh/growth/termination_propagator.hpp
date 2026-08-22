#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology.hpp>

namespace boundary_mesh
{
    class TerminationPropagator
    {
    public:
        static Result<TerminationPropagator, InvalidFaceConstraintState>
        build(
            const GrowthPatch &patch,
            const SurfaceTopology &topology);

        const std::vector<SurfaceFaceId> &neighbors(
            SurfaceFaceId source_face_id) const;

        Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>
        propagateInitial(
            FaceLayerConstraintTable &constraints,
            std::uint32_t max_difference) const;

        Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>
        applyDirectStops(
            FaceLayerConstraintTable &constraints,
            const std::vector<FaceStopEvent> &events,
            std::uint32_t max_difference) const;

        Result<LayerStepResult, InvalidFaceConstraintState>
        filterCandidates(
            const GrowthFront &current_front,
            const LayerStepResult &step,
            const FaceLayerConstraintTable &constraints) const;

    private:
        struct NeighborEntry
        {
            SurfaceFaceId source_face_id{}; // 当前 Patch 源面编号
            std::vector<SurfaceFaceId> neighbors; // 仅共享完整边的 Patch 邻面
        };

        Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>
        propagate(
            FaceLayerConstraintTable &constraints,
            std::uint32_t max_difference) const;

        std::vector<NeighborEntry> entries_; // 按源面编号升序保存的邻接图
    };
}
