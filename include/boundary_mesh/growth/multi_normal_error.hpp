#pragma once

#include <cstddef>
#include <variant>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct MultiNormalInputMismatch
    {
        std::size_t vertex_count{};
        std::size_t face_count{};
    };

    struct NonManifoldIncidentFan { VertexId vertex_id{}; };
    struct DisconnectedIncidentFan { VertexId vertex_id{}; };
    struct InconsistentIncidentFanWinding { VertexId vertex_id{}; };
    struct InvalidMultiNormalTopology { VertexId source_vertex_id{}; };
    struct NonFiniteMultiNormalDisplacement { VertexId source_vertex_id{}; };
    struct DegenerateTriangleTransition { SurfaceFaceId source_face_id{}; };
    struct InvertedTriangleTransition { SurfaceFaceId source_face_id{}; };

    using MultiNormalError = std::variant<
        MultiNormalInputMismatch,
        NonManifoldIncidentFan,
        DisconnectedIncidentFan,
        InconsistentIncidentFanWinding,
        InvalidMultiNormalTopology,
        NonFiniteMultiNormalDisplacement,
        DegenerateTriangleTransition,
        InvertedTriangleTransition>;
}
