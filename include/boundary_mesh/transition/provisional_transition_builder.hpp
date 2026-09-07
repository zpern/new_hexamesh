#pragma once

#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/transition_template_types.hpp>

namespace boundary_mesh
{
    ProvisionalLayerTransitionResult buildProvisionalTransition(
        const GrowthFront &current,
        const GrowthFront &candidate,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets);
}
