#pragma once

#include <cstddef>
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

    struct VertexSplitPlan
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        Scalar original_visibility_cosine{};
        Scalar original_skewness{};
        Scalar selected_skewness{};
        std::vector<VertexId> splitter_neighbors;
        std::vector<SplitBranch> branches;
    };

    Result<std::vector<VertexSplitPlan>, MultiNormalError>
    planMultiNormalSplits(
        const GrowthFront &front,
        const std::vector<IncidentFaceFan> &fans,
        const MultiNormalOptions &options);
}
