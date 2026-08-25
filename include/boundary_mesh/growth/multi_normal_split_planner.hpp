#pragma once

#include <cstddef>
#include <array>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/incident_face_fan.hpp>
#include <boundary_mesh/growth/multi_normal_error.hpp>
#include <boundary_mesh/growth/multi_normal_types.hpp>

namespace boundary_mesh
{
    struct SplitBranch
    {
        std::vector<std::size_t> face_indices;
        Vector3 direction{Vector3::Zero()};
        Scalar visibility_cosine{};
    };

    struct SplitVirtualPoint
    {
        enum class Kind { LocalBranch, FarVertex };
        Kind kind{Kind::LocalBranch};
        std::size_t branch_index{};
        VertexId far_vertex_id{};
    };

    struct SplitActiveTriangle
    {
        std::size_t far_corner{};
        std::array<SplitVirtualPoint, 2> directed_edge;
        Vector3 unit_normal{Vector3::Zero()};
    };

    struct SplitNeighborTriangleChain
    {
        VertexId neighbor_vertex_id{};
        std::vector<SplitActiveTriangle> triangles;
    };

    struct VertexSplitPlan
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        Scalar original_visibility_cosine{};
        Scalar original_skewness{};
        Scalar selected_skewness{};
        std::vector<VertexId> splitter_neighbors;
        std::vector<SplitBranch> branches;
        std::vector<SplitNeighborTriangleChain> neighbor_triangle_chains;
    };

    Result<std::vector<VertexSplitPlan>, MultiNormalError>
    planMultiNormalSplits(
        const GrowthFront &front,
        const std::vector<IncidentFaceFan> &fans,
        const MultiNormalOptions &options);
}
