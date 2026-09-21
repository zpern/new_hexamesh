#pragma once

#include <cstdint>
#include <unordered_map>

#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/transition_template_types.hpp>

namespace boundary_mesh
{
    class ProvisionalTransitionBuildContext
    {
    public:
        ProvisionalTransitionBuildContext(
            const GrowthFront &current,
            const GrowthFront &candidate);

        const GrowthFront &current() const { return current_; }
        const GrowthFront &candidate() const { return candidate_; }
        const auto &currentFaces() const { return current_faces_; }
        const auto &candidateFaces() const { return candidate_faces_; }
        const auto &currentEdgeFaces() const
        { return current_edge_faces_; }
        const auto &currentVertices() const { return current_vertices_; }
        const auto &candidateVertices() const { return candidate_vertices_; }

    private:
        const GrowthFront &current_;
        const GrowthFront &candidate_;
        std::unordered_map<SurfaceFaceId, std::size_t> current_faces_;
        std::unordered_map<SurfaceFaceId, std::size_t> candidate_faces_;
        std::unordered_map<std::uint64_t, std::vector<SurfaceFaceId>>
            current_edge_faces_;
        std::unordered_map<std::uint64_t, std::size_t> current_vertices_;
        std::unordered_map<std::uint64_t, std::size_t> candidate_vertices_;
    };

    ProvisionalLayerTransitionResult buildProvisionalTransition(
        const ProvisionalTransitionBuildContext &context,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points = {},
        const ExternalPatchControls &external_controls = {},
        const std::vector<SurfaceFaceId> &terminal_candidate_faces = {});

    ProvisionalLayerTransitionResult buildProvisionalTransition(
        const GrowthFront &current,
        const GrowthFront &candidate,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points = {},
        const ExternalPatchControls &external_controls = {},
        const std::vector<SurfaceFaceId> &terminal_candidate_faces = {});

    ProvisionalLayerTransitionResult buildProvisionalExternalPatches(
        const ProvisionalTransitionBuildContext &context,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::vector<SurfaceFaceId> &selected_external_faces,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points = {},
        const ExternalPatchControls &external_controls = {},
        const std::vector<SurfaceFaceId> &terminal_candidate_faces = {});

    ProvisionalLayerTransitionResult buildProvisionalExternalPatches(
        const GrowthFront &current,
        const GrowthFront &candidate,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::vector<SurfaceFaceId> &selected_external_faces,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points = {},
        const ExternalPatchControls &external_controls = {},
        const std::vector<SurfaceFaceId> &terminal_candidate_faces = {});
}
