#include <algorithm>

#include <boundary_mesh/transition/incremental_transition_collision_state.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool sameSlidingColumns(
            const std::shared_ptr<const SlidingColumnContext> &left,
            const std::shared_ptr<const SlidingColumnContext> &right)
        {
            if (!left || !right) return left == right;
            return left->low_points == right->low_points &&
                left->high_points == right->high_points &&
                left->low_region_ids == right->low_region_ids &&
                left->high_region_ids == right->high_region_ids;
        }

        bool sameTriangle(
            const OwnedBoundaryTriangle &left,
            const OwnedBoundaryTriangle &right)
        {
            bool same_keys = true;
            for (std::size_t index = 0; index < 3; ++index)
                same_keys = same_keys &&
                    left.vertex_keys[index].source_vertex_id ==
                        right.vertex_keys[index].source_vertex_id &&
                    left.vertex_keys[index].layer ==
                        right.vertex_keys[index].layer &&
                    left.vertex_keys[index].branch_id ==
                        right.vertex_keys[index].branch_id;
            return left.points == right.points && same_keys &&
                layerBoundaryOwnerKey(left.owner) ==
                    layerBoundaryOwnerKey(right.owner) &&
                left.owner.rollback_high_faces ==
                    right.owner.rollback_high_faces &&
                left.vertex_sliding_region_ids ==
                    right.vertex_sliding_region_ids &&
                left.physical_edge_mask == right.physical_edge_mask &&
                left.complete_face_exemption_regions ==
                    right.complete_face_exemption_regions &&
                sameSlidingColumns(
                    left.sliding_columns, right.sliding_columns);
        }

        std::pair<std::uint32_t, std::uint32_t> contactKey(
            std::uint32_t first, std::uint32_t second)
        {
            if (second < first) std::swap(first, second);
            return {first, second};
        }
    }

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

    IncrementalTransitionCollisionState::BuildResult
    IncrementalTransitionCollisionState::build(
        const TransitionBoundaryInput &input)
    {
        auto state = buildBoundary(input);
        if (!state.hasValue()) return state;
        const auto collisions = state.value().initializeCollisions(input);
        if (!collisions.hasValue())
            return BuildResult::failure(collisions.error());
        return state;
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

    IncrementalTransitionCollisionState::UpdateResult
    IncrementalTransitionCollisionState::update(
        const TransitionBoundaryInput &next,
        const std::vector<LayerBoundaryOwnerKey> &changed_owners)
    {
        IncrementalTransitionCollisionState working = *this;
        const auto boundary = working.replaceBoundary(next, changed_owners);
        if (!boundary.hasValue()) return boundary;

        const bool environment_changed =
            working.original_surface_ != next.original_surface ||
            working.historical_boundary_ != next.historical_boundary ||
            working.historical_index_ != next.historical_index ||
            working.prior_transition_boundary_ !=
                next.prior_transition_boundary ||
            working.sliding_surface_ != next.sliding_surface;
        if (environment_changed)
        {
            auto rebuilt = build(next);
            if (!rebuilt.hasValue())
                return UpdateResult::failure(rebuilt.error());
            *this = std::move(rebuilt.value());
            return UpdateResult::success(std::monostate{});
        }

        std::map<TransitionTriangleKey, OwnedBoundaryTriangle> next_exposed;
        for (const OwnedBoundaryTriangle &triangle : working.exposed_boundary_)
            next_exposed.emplace(transitionTriangleKey(triangle), triangle);

        std::set<std::uint32_t> removed_ids;
        std::vector<OwnedBoundaryTriangle> additions;
        for (const auto &[key, active] : working.active_primitives_)
        {
            const auto found = next_exposed.find(key);
            if (found == next_exposed.end() ||
                !sameTriangle(active.triangle, found->second))
                removed_ids.insert(active.id);
        }
        for (const auto &[key, triangle] : next_exposed)
        {
            const auto found = working.active_primitives_.find(key);
            if (found == working.active_primitives_.end() ||
                !sameTriangle(found->second.triangle, triangle))
                additions.push_back(triangle);
        }

        for (const std::uint32_t id : removed_ids)
        {
            const auto erased = working.collision_index_->eraseGroup(
                static_cast<CollisionGroupId>(id) + 2);
            if (!erased.hasValue())
                return UpdateResult::failure(
                    TransitionBoundaryError{erased.error()});
            const auto key = working.primitive_keys_.find(id);
            if (key != working.primitive_keys_.end())
            {
                working.active_primitives_.erase(key->second);
                working.primitive_keys_.erase(key);
            }
        }
        for (auto iterator = working.contacts_.begin();
             iterator != working.contacts_.end();)
            if (removed_ids.find(iterator->first) != removed_ids.end() ||
                removed_ids.find(iterator->second) != removed_ids.end())
                iterator = working.contacts_.erase(iterator);
            else
                ++iterator;

        std::vector<std::uint32_t> added_ids;
        for (const OwnedBoundaryTriangle &triangle : additions)
        {
            const std::uint32_t id = working.next_primitive_id_++;
            auto collision = makeTransitionCollisionTriangle(triangle, id);
            const auto inserted = working.collision_index_->insertGroup({
                static_cast<CollisionGroupId>(id) + 2, {collision}});
            if (!inserted.hasValue())
                return UpdateResult::failure(
                    TransitionBoundaryError{inserted.error()});
            const auto static_hit =
                working.static_obstacle_context_->intersects(triangle);
            if (!static_hit.hasValue())
                return UpdateResult::failure(static_hit.error());
            const TransitionTriangleKey key =
                transitionTriangleKey(triangle);
            working.active_primitives_[key] =
                {id, triangle, static_hit.value()};
            working.primitive_keys_[id] = key;
            added_ids.push_back(id);
            ++working.diagnostics_.static_obstacle_queries;
        }
        for (const std::uint32_t id : added_ids)
        {
            const auto key = working.primitive_keys_.find(id);
            const auto active = working.active_primitives_.find(key->second);
            const CollisionTriangle query = makeTransitionCollisionTriangle(
                active->second.triangle, id);
            for (const CollisionPrimitiveId contact :
                 working.collision_index_->queryIllegalContacts(
                     query, static_cast<CollisionGroupId>(id) + 2))
            {
                const std::uint32_t other =
                    working.collision_index_->primitive(contact).owner_id;
                working.contacts_.insert(contactKey(id, other));
            }
            ++working.diagnostics_.self_collision_queries;
        }
        working.materializeCollisionReport();
        *this = std::move(working);
        return UpdateResult::success(std::monostate{});
    }

    Result<std::monostate, TransitionBoundaryError>
    IncrementalTransitionCollisionState::initializeCollisions(
        const TransitionBoundaryInput &input)
    {
        original_surface_ = input.original_surface;
        historical_boundary_ = input.historical_boundary;
        historical_index_ = input.historical_index;
        prior_transition_boundary_ = input.prior_transition_boundary;
        sliding_surface_ = input.sliding_surface;
        active_primitives_.clear();
        primitive_keys_.clear();
        contacts_.clear();
        next_primitive_id_ = 0;
        auto static_context = TransitionStaticObstacleContext::build(input);
        if (!static_context.hasValue())
            return Result<std::monostate,
                TransitionBoundaryError>::failure(static_context.error());
        static_obstacle_context_ = std::move(static_context.value());
        std::vector<CollisionPrimitiveGroup> groups;
        for (const OwnedBoundaryTriangle &triangle : exposed_boundary_)
        {
            const std::uint32_t id = next_primitive_id_++;
            groups.push_back({
                static_cast<CollisionGroupId>(id) + 2,
                {makeTransitionCollisionTriangle(triangle, id)}});
            const auto static_hit =
                static_obstacle_context_->intersects(triangle);
            if (!static_hit.hasValue())
                return Result<std::monostate,
                    TransitionBoundaryError>::failure(static_hit.error());
            const TransitionTriangleKey key =
                transitionTriangleKey(triangle);
            active_primitives_[key] = {id, triangle, static_hit.value()};
            primitive_keys_[id] = key;
            ++diagnostics_.static_obstacle_queries;
        }
        auto index = IncrementalCollisionIndex::build(std::move(groups));
        if (!index.hasValue())
            return Result<std::monostate,
                TransitionBoundaryError>::failure(
                    TransitionBoundaryError{index.error()});
        collision_index_ = std::move(index.value());
        for (const auto &[key, active] : active_primitives_)
        {
            (void)key;
            const CollisionTriangle query = makeTransitionCollisionTriangle(
                active.triangle, active.id);
            for (const CollisionPrimitiveId contact :
                 collision_index_->queryIllegalContacts(
                     query,
                     static_cast<CollisionGroupId>(active.id) + 2))
            {
                const std::uint32_t other =
                    collision_index_->primitive(contact).owner_id;
                contacts_.insert(contactKey(active.id, other));
            }
            ++diagnostics_.self_collision_queries;
        }
        diagnostics_.full_collision_builds = 1;
        materializeCollisionReport();
        return Result<std::monostate,
            TransitionBoundaryError>::success(std::monostate{});
    }

    void IncrementalTransitionCollisionState::materializeCollisionReport()
    {
        collision_report_ = {};
        std::set<std::uint32_t> colliding_ids;
        for (const auto &[key, active] : active_primitives_)
        {
            (void)key;
            if (active.static_obstacle_hit)
                colliding_ids.insert(active.id);
        }
        for (const auto &contact : contacts_)
        {
            colliding_ids.insert(contact.first);
            colliding_ids.insert(contact.second);
        }
        std::set<LayerBoundaryOwnerKey> owners;
        for (const std::uint32_t id : colliding_ids)
        {
            const auto key = primitive_keys_.find(id);
            if (key == primitive_keys_.end()) continue;
            const auto active = active_primitives_.find(key->second);
            if (active == active_primitives_.end()) continue;
            const LayerBoundaryOwner &owner = active->second.triangle.owner;
            if (owners.insert(layerBoundaryOwnerKey(owner)).second)
                collision_report_.colliding_owners.push_back(owner);
            collision_report_.rollback_faces.insert(
                collision_report_.rollback_faces.end(),
                owner.rollback_high_faces.begin(),
                owner.rollback_high_faces.end());
        }
        std::sort(collision_report_.rollback_faces.begin(),
                  collision_report_.rollback_faces.end());
        collision_report_.rollback_faces.erase(std::unique(
            collision_report_.rollback_faces.begin(),
            collision_report_.rollback_faces.end()),
            collision_report_.rollback_faces.end());
    }
}
