#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <boundary_mesh/transition/incremental_transition_templates.hpp>

namespace boundary_mesh
{
    struct QuadHighNeighbor
    {
        std::size_t local_edge{};
        SurfaceFaceId neighbor_face_id{};
    };

    struct QuadHighNeighborSelectionInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t low_layer{};
        std::vector<QuadHighNeighbor> high_neighbors;
        std::array<VertexId, 4> low{};
        std::array<VertexId, 4> high{};
        const std::vector<Point3> *mesh_vertices{};
        Scalar length_tolerance{1e-12};
    };

    struct QuadHighNeighborSelection
    {
        std::vector<std::size_t> retained_local_edges;
        std::vector<SurfaceFaceId> retained_neighbor_faces;
        std::vector<SurfaceFaceId> suppressed_neighbor_faces;
        std::optional<QuadDiagonal> required_low_diagonal;
        Scalar worst_skewness{};
    };

    using QuadHighNeighborSelectionResult = Result<
        QuadHighNeighborSelection, TransitionTemplateError>;

    QuadHighNeighborSelectionResult selectQuadHighNeighbors(
        const QuadHighNeighborSelectionInput &input);
}
