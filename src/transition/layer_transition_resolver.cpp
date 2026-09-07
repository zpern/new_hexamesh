#include <algorithm>
#include <utility>

#include <boundary_mesh/transition/layer_transition_resolver.hpp>

namespace boundary_mesh
{
    namespace
    {
        GrowthFront retainFaces(
            const GrowthFront &front,
            const std::vector<SurfaceFaceId> &retained)
        {
            GrowthFront result = front;
            result.faces.clear();
            result.source_face_ids.clear();
            for (std::size_t index = 0;
                 index < front.source_face_ids.size(); ++index)
            {
                if (!std::binary_search(
                        retained.begin(), retained.end(),
                        front.source_face_ids[index]))
                    continue;
                result.faces.push_back(front.faces[index]);
                result.source_face_ids.push_back(
                    front.source_face_ids[index]);
            }
            return result;
        }
    }

    Result<StableLayerTransition, LayerTransitionError>
    LayerTransitionResolver::resolve(
        const LayerTransitionInput &input) const
    {
        using ResolveResult = Result<
            StableLayerTransition, LayerTransitionError>;
        if (!input.build_provisional)
            return ResolveResult::failure(LayerTransitionError{
                TransitionCoordinationError{
                    MissingTransitionFaceState{0}}});

        LayerFaceSets face_sets = input.face_sets;
        std::vector<SurfaceFaceId> retained =
            input.candidate_front.source_face_ids;
        std::sort(retained.begin(), retained.end());
        retained.erase(std::unique(retained.begin(), retained.end()),
                       retained.end());
        std::uint32_t iterations = 1;
        TransitionBoundaryChecker checker;

        while (true)
        {
            const auto suppression = applyCornerSuppression({
                input.current_front,
                retainFaces(input.candidate_front, retained),
                face_sets,
                input.completed_layer,
                input.length_tolerance});
            if (!suppression.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    suppression.error()});
            face_sets = suppression.value().face_sets;
            retained = suppression.value().retained_high_faces;

            auto provisional = input.build_provisional(
                retained, face_sets);
            if (!provisional.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    provisional.error()});
            provisional.value().boundary.original_surface =
                input.original_surface.has_value()
                    ? &*input.original_surface
                    : nullptr;
            provisional.value().boundary.historical_boundary =
                input.historical_boundary;

            const auto rollback = checker.findRollbackFaces(
                provisional.value().boundary);
            if (!rollback.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    rollback.error()});
            if (rollback.value().empty())
            {
                const auto exposed = checker.assembleExposedBoundary(
                    provisional.value().boundary);
                if (!exposed.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        exposed.error()});
                StableLayerTransition stable;
                stable.face_sets = std::move(face_sets);
                stable.retained_high_faces = std::move(retained);
                stable.exposed_boundary = exposed.value();
                stable.resolved_topology = std::move(
                    provisional.value().resolved_topology);
                stable.iterations = iterations;
                stable.all_top_faces_are_triangles =
                    provisional.value().all_top_faces_are_triangles;
                return ResolveResult::success(std::move(stable));
            }

            bool changed = false;
            for (const SurfaceFaceId id : rollback.value())
            {
                const auto position = std::lower_bound(
                    retained.begin(), retained.end(), id);
                if (position == retained.end() || *position != id)
                    continue;
                retained.erase(position);
                addCollisionRollback(
                    face_sets,
                    {id, input.completed_layer,
                     StopOrigin::TransitionCollision});
                changed = true;
            }
            if (!changed)
                return ResolveResult::failure(LayerTransitionError{
                    TransitionCoordinationError{
                        MissingTransitionFaceState{
                            rollback.value().front()}}});
            ++iterations;
        }
    }
}
