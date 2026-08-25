#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/mesh/mesh_volume.hpp>

namespace boundary_mesh
{
    struct MultiNormalDebugOutput
    {
        bool enabled{};
        std::filesystem::path directory;
        std::string transition_filename{"multi_normal_transition.vtk"};
        std::string front_filename{"multi_normal_front.vtk"};
    };

    struct MultiNormalOptions
    {
        bool enabled{false};
        Scalar transition_height{};
        Scalar split_skewness_threshold{0.80};
        Scalar plane_skewness_threshold{-0.10};
        Scalar convex_skewness_threshold{0.20};
        std::size_t maximum_strategy_count{20};
        MultiNormalDebugOutput debug_output;
    };

    struct SplitVertexMapping
    {
        VertexId transformed_vertex_id{};
        VertexId source_vertex_id{};
        std::uint32_t branch_id{};
    };

    struct TransitionFaceOrigin
    {
        std::size_t transformed_face_index{};
        std::vector<SurfaceFaceId> source_face_ids;
        std::vector<VertexId> source_vertex_ids;
    };

    struct OmittedQuadTransition
    {
        SurfaceFaceId source_face_id{};
        std::array<VertexId, 4> topology_vertex_ids{};
        std::array<VertexId, 4> bottom_vertex_ids{};
        std::array<bool, 4> moved_corners{};
    };

    struct MultiNormalTransitionResult
    {
        bool applied{};
        VolumeMesh transition_cells;
        GrowthFront transformed_front;
        std::vector<SplitVertexMapping> vertex_mapping;
        std::vector<TransitionFaceOrigin> transition_face_origins;
        std::vector<OmittedQuadTransition> omitted_quad_transitions;
        std::vector<VertexId> transformed_front_volume_vertex_ids;
    };
}
