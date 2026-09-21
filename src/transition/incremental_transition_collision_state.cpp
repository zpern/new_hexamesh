#include <algorithm>

#include <boundary_mesh/transition/incremental_transition_collision_state.hpp>

namespace boundary_mesh
{
    Result<std::monostate, TransitionBoundaryError>
    IncrementalTransitionCollisionState::validateDiagonals(
        const TransitionBoundaryInput &input)
    {
        using ValidationResult = Result<
            std::monostate, TransitionBoundaryError>;
        for (std::size_t first = 0;
             first < input.diagonal_requirements.size(); ++first)
            for (std::size_t second = first + 1;
                 second < input.diagonal_requirements.size(); ++second)
            {
                const auto &left = input.diagonal_requirements[first];
                const auto &right = input.diagonal_requirements[second];
                if (left.key.source_face_id == right.key.source_face_id &&
                    left.key.layer == right.key.layer &&
                    left.diagonal != right.diagonal)
                    return ValidationResult::failure(
                        TransitionBoundaryError{
                            ConflictingLayerQuadDiagonal{
                                left.key, left.diagonal, right.diagonal}});
            }
        return ValidationResult::success(std::monostate{});
    }

    void IncrementalTransitionCollisionState::materializeExposed()
    {
        exposed_boundary_.clear();
        for (const auto &[key, contributions] : buckets_)
        {
            (void)key;
            if (contributions.size() % 2 == 1)
                exposed_boundary_.push_back(contributions.front());
        }
    }

    IncrementalTransitionCollisionState::BuildResult
    IncrementalTransitionCollisionState::buildBoundary(
        const TransitionBoundaryInput &input)
    {
        const auto validated = validateDiagonals(input);
        if (!validated.hasValue())
            return BuildResult::failure(validated.error());

        IncrementalTransitionCollisionState state;
        state.diagonal_requirements_ = input.diagonal_requirements;
        for (const OwnedBoundaryTriangle &triangle :
             input.candidate_triangles)
        {
            const TransitionTriangleKey triangle_key =
                transitionTriangleKey(triangle);
            state.buckets_[triangle_key].push_back(triangle);
            state.owner_keys_[layerBoundaryOwnerKey(triangle.owner)]
                .insert(triangle_key);
        }
        state.materializeExposed();
        state.diagnostics_.exposed_inserts =
            state.exposed_boundary_.size();
        return BuildResult::success(std::move(state));
    }

    IncrementalTransitionCollisionState::UpdateResult
    IncrementalTransitionCollisionState::replaceBoundary(
        const TransitionBoundaryInput &next,
        const std::vector<LayerBoundaryOwnerKey> &changed_owners)
    {
        const auto validated = validateDiagonals(next);
        if (!validated.hasValue())
            return UpdateResult::failure(validated.error());

        IncrementalTransitionCollisionState working = *this;
        std::set<LayerBoundaryOwnerKey> changed(
            changed_owners.begin(), changed_owners.end());
        std::set<TransitionTriangleKey> affected;
        for (const LayerBoundaryOwnerKey &owner : changed)
        {
            const auto found = working.owner_keys_.find(owner);
            if (found != working.owner_keys_.end())
                affected.insert(found->second.begin(), found->second.end());
        }
        for (const OwnedBoundaryTriangle &triangle : next.candidate_triangles)
            if (changed.find(layerBoundaryOwnerKey(triangle.owner)) !=
                changed.end())
                affected.insert(transitionTriangleKey(triangle));

        std::map<TransitionTriangleKey,
                 std::vector<OwnedBoundaryTriangle>> replacements;
        for (const OwnedBoundaryTriangle &triangle : next.candidate_triangles)
        {
            const TransitionTriangleKey key =
                transitionTriangleKey(triangle);
            if (affected.find(key) != affected.end())
                replacements[key].push_back(triangle);
        }

        IncrementalTransitionCollisionDiagnostics diagnostics;
        diagnostics.changed_owners = changed.size();
        diagnostics.affected_triangle_keys = affected.size();
        for (const TransitionTriangleKey &key : affected)
        {
            const auto old = working.buckets_.find(key);
            const bool old_exposed = old != working.buckets_.end() &&
                old->second.size() % 2 == 1;
            const auto replacement = replacements.find(key);
            const bool new_exposed = replacement != replacements.end() &&
                replacement->second.size() % 2 == 1;
            if (old_exposed && !new_exposed) ++diagnostics.exposed_erases;
            else if (!old_exposed && new_exposed)
                ++diagnostics.exposed_inserts;
            else if (old_exposed && new_exposed)
                ++diagnostics.exposed_replacements;

            if (replacement == replacements.end() ||
                replacement->second.empty())
                working.buckets_.erase(key);
            else
                working.buckets_[key] = replacement->second;
        }

        working.owner_keys_.clear();
        for (const OwnedBoundaryTriangle &triangle : next.candidate_triangles)
            working.owner_keys_[layerBoundaryOwnerKey(triangle.owner)]
                .insert(transitionTriangleKey(triangle));
        working.diagonal_requirements_ = next.diagonal_requirements;
        working.diagnostics_ = diagnostics;
        working.materializeExposed();
        *this = std::move(working);
        return UpdateResult::success(std::monostate{});
    }
}
