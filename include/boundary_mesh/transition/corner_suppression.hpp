#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/transition/incremental_transition_types.hpp>

namespace boundary_mesh
{
    struct CornerSuppressionInput
    {
        GrowthFront current_front;
        GrowthFront candidate_front;
        LayerFaceSets face_sets;
        std::uint32_t completed_layer{};
        Scalar length_tolerance{1e-12};
    };

    struct CornerSuppressionView
    {
        const GrowthFront &current_front;
        const GrowthFront &candidate_front;
        const std::vector<SurfaceFaceId> &retained_high_faces;
        const LayerFaceSets &face_sets;
        std::uint32_t completed_layer{};
        Scalar length_tolerance{1e-12};
    };

    struct CornerSuppressionResult
    {
        LayerFaceSets face_sets;
        std::vector<SurfaceFaceId> retained_high_faces;
        std::vector<SurfaceFaceId> removed_high_faces;
    };

    class CornerSuppressionContext
    {
    public:
        CornerSuppressionContext(
            const GrowthFront &current_front,
            const GrowthFront &candidate_front);

    private:
        friend Result<CornerSuppressionResult, TransitionCoordinationError>
        applyCornerSuppression(
            const CornerSuppressionView &,
            const CornerSuppressionContext &);

        const GrowthFront *current_front_{};
        const GrowthFront *candidate_front_{};
        std::unordered_map<SurfaceFaceId, std::size_t> face_indices_;
        std::unordered_map<std::uint64_t, std::vector<SurfaceFaceId>>
            edge_faces_;
        std::unordered_map<VertexId, std::vector<SurfaceFaceId>>
            vertex_faces_;
        std::unordered_map<VertexId, Point3> candidate_points_;
    };

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(const CornerSuppressionInput &input);

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(const CornerSuppressionView &input);

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(
        const CornerSuppressionView &input,
        const CornerSuppressionContext &context);
}
