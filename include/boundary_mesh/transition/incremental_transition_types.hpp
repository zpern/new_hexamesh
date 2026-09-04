#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    enum class StopOrigin
    {
        RequestedLimit,
        Quality,
        Collision,
        SlidingProjection,
        IsotropicStop,
        AcceptedStop,
        CornerSuppression,
        TransitionCollision
    };

    struct LayerStopState
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t completed_layer{};
        StopOrigin origin{};
    };

    struct LayerFaceSets
    {
        std::vector<SurfaceFaceId> corner_suppression_seeds;
        std::vector<SurfaceFaceId> transition_low_faces;
        std::vector<LayerStopState> states;
    };

    namespace detail
    {
        inline void insertSortedUnique(
            std::vector<SurfaceFaceId> &ids,
            SurfaceFaceId id)
        {
            const auto position = std::lower_bound(
                ids.begin(), ids.end(), id);
            if (position == ids.end() || *position != id)
                ids.insert(position, id);
        }

        inline void insertState(
            std::vector<LayerStopState> &states,
            const LayerStopState &state)
        {
            const auto position = std::lower_bound(
                states.begin(), states.end(), state.source_face_id,
                [](const LayerStopState &value, SurfaceFaceId id)
                { return value.source_face_id < id; });
            if (position == states.end() ||
                position->source_face_id != state.source_face_id)
                states.insert(position, state);
        }
    }

    inline void addInitialStop(
        LayerFaceSets &sets,
        const LayerStopState &state)
    {
        detail::insertSortedUnique(
            sets.corner_suppression_seeds, state.source_face_id);
        detail::insertSortedUnique(
            sets.transition_low_faces, state.source_face_id);
        detail::insertState(sets.states, state);
    }

    inline void addCornerSuppressedFace(
        LayerFaceSets &sets,
        const LayerStopState &state)
    {
        detail::insertSortedUnique(
            sets.transition_low_faces, state.source_face_id);
        detail::insertState(sets.states, state);
    }

    inline void addCollisionRollback(
        LayerFaceSets &sets,
        const LayerStopState &state)
    {
        detail::insertSortedUnique(
            sets.corner_suppression_seeds, state.source_face_id);
        detail::insertSortedUnique(
            sets.transition_low_faces, state.source_face_id);
        detail::insertState(sets.states, state);
    }
}
