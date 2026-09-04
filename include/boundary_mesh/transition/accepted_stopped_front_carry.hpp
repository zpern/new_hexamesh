#pragma once

#include <vector>

#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/transition/incremental_transition_types.hpp>

namespace boundary_mesh
{
    struct AcceptedStoppedFrontCarry
    {
        GrowthFront front;
        std::vector<LayerStopState> stops;
    };

    AcceptedStoppedFrontCarry carryAcceptedStoppedFaces(
        const GrowthFront &accepted_front,
        const std::vector<FaceStopEvent> &accepted_stops,
        const std::vector<SurfaceFaceId> &retained_faces);

    AcceptedStoppedFrontCarry retainAcceptedFaces(
        const GrowthFront &accepted_front,
        const std::vector<FaceStopEvent> &accepted_stops,
        const std::vector<SurfaceFaceId> &retained_faces);

    AcceptedStoppedFrontCarry carryFacesMissingFromActive(
        const AcceptedStoppedFrontCarry &previously_accepted,
        const GrowthFront &active_front);

    GrowthFront mergeWithCarriedStoppedFaces(
        const GrowthFront &active_front,
        const AcceptedStoppedFrontCarry &carry);

    void addCarriedStops(
        LayerFaceSets &sets,
        const AcceptedStoppedFrontCarry &carry);
}
