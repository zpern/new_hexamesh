#pragma once

#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/transition_template_types.hpp>

namespace boundary_mesh
{
    ProvisionalLayerTransitionResult buildProvisionalTransition(
        const GrowthFront &current,
        const GrowthFront &candidate,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points = {},
        const ExternalPatchControls &external_controls = {},
        const std::vector<SurfaceFaceId> &terminal_candidate_faces = {});
}
