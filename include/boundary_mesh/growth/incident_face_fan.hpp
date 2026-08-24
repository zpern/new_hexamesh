#pragma once

#include <cstddef>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/multi_normal_error.hpp>

namespace boundary_mesh
{
    struct IncidentFaceSector
    {
        std::size_t face_index{};
        VertexId previous_vertex{};
        VertexId next_vertex{};
        Vector3 unit_normal{Vector3::Zero()};
    };

    struct IncidentFaceFan
    {
        VertexId center_vertex{};
        bool closed{};
        std::vector<IncidentFaceSector> sectors;
    };

    Result<std::vector<IncidentFaceFan>, MultiNormalError>
    buildIncidentFaceFans(
        const GrowthFront &front,
        const FrontEvaluation &evaluation);
}
