#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>

#include <boundary_mesh/transition/incremental_transition_collision_state.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

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

        bool traceFace(SurfaceFaceId id)
        {
            const char *value = std::getenv("BOUNDARY_MESH_TRACE_FACE");
            if (value == nullptr) return false;
            while (*value != '\0')
            {
                char *end = nullptr;
                const auto parsed = std::strtoull(value, &end, 10);
                if (end != value && parsed == id) return true;
                if (end == value) break;
                value = end;
                while (*value == ',' || *value == ';' || *value == ' ')
                    ++value;
            }
            return false;
        }

        const char *roleName(BoundaryOwnerRole role)
        {
            switch (role)
            {
                case BoundaryOwnerRole::RegularCandidate: return "regular";
                case BoundaryOwnerRole::TopCap: return "topcap";
                case BoundaryOwnerRole::SideTransition: return "side";
                case BoundaryOwnerRole::ExternalPatch: return "external";
            }
            return "unknown";
        }

    }

    Result<std::monostate, TransitionBoundaryError>
    IncrementalTransitionCollisionState::validateDiagonals(
        const TransitionBoundaryInput &input)
    {
        using ValidationResult = Result<
            std::monostate, TransitionBoundaryError>;
        std::map<std::pair<SurfaceFaceId, std::uint32_t>, QuadDiagonal> seen;
        for (const auto &right : input.diagonal_requirements)
        {
                const auto [found, inserted] = seen.emplace(
                    std::make_pair(right.key.source_face_id, right.key.layer), right.diagonal);
                if (!inserted && found->second != right.diagonal)
                    return ValidationResult::failure(
                        TransitionBoundaryError{
                            ConflictingLayerQuadDiagonal{
                                right.key, found->second, right.diagonal}});
        }
        return ValidationResult::success(std::monostate{});
    }

    void IncrementalTransitionCollisionState::materializeExposed()
    {
        exposed_dirty_ = false;
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

        IncrementalTransitionCollisionState working;
        working.buckets_ = buckets_;
        working.owner_keys_ = owner_keys_;
        working.diagonal_requirements_ = diagonal_requirements_;
        working.exposed_boundary_ = exposed_boundary_;
        const auto replaced = working.replaceBoundaryInPlace(
            next, changed_owners, next.candidate_triangles);
        if (!replaced.hasValue()) return replaced;
        buckets_ = std::move(working.buckets_);
        owner_keys_ = std::move(working.owner_keys_);
        diagonal_requirements_ =
            std::move(working.diagonal_requirements_);
        exposed_boundary_ = std::move(working.exposed_boundary_);
        diagnostics_ = working.diagnostics_;
        return UpdateResult::success(std::monostate{});
    }

    IncrementalTransitionCollisionState::UpdateResult
    IncrementalTransitionCollisionState::replaceBoundaryInPlace(
        const TransitionBoundaryInput &next,
        const std::vector<LayerBoundaryOwnerKey> &changed_owners,
        const std::vector<OwnedBoundaryTriangle> &changed_triangles)
    {
        std::set<LayerBoundaryOwnerKey> changed(
            changed_owners.begin(), changed_owners.end());
        std::set<TransitionTriangleKey> affected;
        for (const LayerBoundaryOwnerKey &owner : changed)
        {
            const auto found = owner_keys_.find(owner);
            if (found != owner_keys_.end())
                affected.insert(found->second.begin(), found->second.end());
        }
        for (const OwnedBoundaryTriangle &triangle : changed_triangles)
            if (changed.find(layerBoundaryOwnerKey(triangle.owner)) !=
                changed.end())
                affected.insert(transitionTriangleKey(triangle));

        std::map<TransitionTriangleKey,
                 std::vector<OwnedBoundaryTriangle>> replacements;
        for (const OwnedBoundaryTriangle &triangle : changed_triangles)
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
            const auto old = buckets_.find(key);
            const bool old_exposed = old != buckets_.end() &&
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
                buckets_.erase(key);
            else
                buckets_[key] = replacement->second;
        }

        for (const LayerBoundaryOwnerKey &owner : changed)
            owner_keys_.erase(owner);
        for (const OwnedBoundaryTriangle &triangle : changed_triangles)
        {
            const LayerBoundaryOwnerKey owner =
                layerBoundaryOwnerKey(triangle.owner);
            if (changed.find(owner) != changed.end())
                owner_keys_[owner].insert(transitionTriangleKey(triangle));
        }
        diagonal_requirements_ = next.diagonal_requirements;
        diagnostics_ = diagnostics;
        materializeExposed();
        return UpdateResult::success(std::monostate{});
    }

    IncrementalTransitionCollisionState::UpdateResult
    IncrementalTransitionCollisionState::update(
        const TransitionBoundaryInput &next,
        const std::vector<LayerBoundaryOwnerKey> &changed_owners)
    {
        return update(next, changed_owners, next.candidate_triangles);
    }

    IncrementalTransitionCollisionState::UpdateResult
    IncrementalTransitionCollisionState::update(
        const TransitionBoundaryInput &next,
        const std::vector<LayerBoundaryOwnerKey> &changed_owners,
        const std::vector<OwnedBoundaryTriangle> &changed_triangles)
    {
        const auto started = std::chrono::steady_clock::now();
        const auto validated = validateDiagonals(next);
        if (!validated.hasValue())
            return UpdateResult::failure(validated.error());
        const bool environment_changed =
            original_surface_ != next.original_surface ||
            historical_boundary_ != next.historical_boundary ||
            historical_index_ != next.historical_index ||
            historical_index_includes_transition_ !=
                next.historical_index_includes_transition ||
            prior_transition_boundary_ !=
                next.prior_transition_boundary ||
            sliding_surface_ != next.sliding_surface ||
            regular_candidate_geometry_prevalidated_ !=
                next.regular_candidate_geometry_prevalidated;
        if (environment_changed)
        {
            auto rebuilt = build(next);
            if (!rebuilt.hasValue())
                return UpdateResult::failure(rebuilt.error());
            *this = std::move(rebuilt.value());
            return UpdateResult::success(std::monostate{});
        }

        // Stage only affected buckets. Shared keys include unchanged owners so
        // parity and the deterministic representative remain identical to a full build.
        std::set<LayerBoundaryOwnerKey> changed(changed_owners.begin(), changed_owners.end());
        std::set<TransitionTriangleKey> affected;
        IncrementalTransitionCollisionState next_boundary;
        for (const auto &owner : changed)
        {
            const auto found = owner_keys_.find(owner);
            if (found == owner_keys_.end()) continue;
            next_boundary.owner_keys_.emplace(owner, found->second);
            affected.insert(found->second.begin(), found->second.end());
        }
        for (const auto &triangle : changed_triangles)
            if (changed.count(layerBoundaryOwnerKey(triangle.owner)))
                affected.insert(transitionTriangleKey(triangle));
        for (const auto &key : affected)
        {
            const auto found = buckets_.find(key);
            if (found != buckets_.end()) next_boundary.buckets_.emplace(key, found->second);
        }
        const auto copied = std::chrono::steady_clock::now();
        const auto boundary = next_boundary.replaceBoundaryInPlace(
            next, changed_owners, changed_triangles);
        if (!boundary.hasValue()) return boundary;
        const auto replaced = std::chrono::steady_clock::now();
        next_boundary.diagnostics_.staged_exposed_triangles =
            next_boundary.exposed_boundary_.size();

        std::set<std::uint32_t> removed_ids;
        std::vector<OwnedBoundaryTriangle> additions;
        for (const auto &key : affected)
        {
            const auto old = active_primitives_.find(key);
            if (old == active_primitives_.end()) continue;
            const auto &active = old->second;
            const auto bucket = next_boundary.buckets_.find(key);
            const bool remains_exposed =
                bucket != next_boundary.buckets_.end() &&
                bucket->second.size() % 2 == 1;
            if (!remains_exposed ||
                !sameTriangle(active.triangle, bucket->second.front()))
                removed_ids.insert(active.id);
        }
        for (const OwnedBoundaryTriangle &triangle :
             next_boundary.exposed_boundary_)
        {
            const TransitionTriangleKey key = transitionTriangleKey(triangle);
            const auto found = active_primitives_.find(key);
            if (found == active_primitives_.end() ||
                !sameTriangle(found->second.triangle, triangle))
                additions.push_back(triangle);
        }
        const auto diffed = std::chrono::steady_clock::now();

        if (additions.size() >
            std::numeric_limits<std::uint32_t>::max() - next_primitive_id_)
            return UpdateResult::failure(TransitionBoundaryError{
                SpatialError::PrimitiveIdOverflow});
        std::vector<bool> static_hits;
        static_hits.reserve(additions.size());
        TransitionStaticObstacleQueryScratch obstacle_scratch;
        for (const OwnedBoundaryTriangle &triangle : additions)
        {
            const CollisionTriangle collision =
                makeTransitionCollisionTriangle(triangle, 0);
            const auto bounds = makeAabb(
                collision.points[0], collision.points[1],
                collision.points[2]);
            if (!bounds.hasValue())
                return UpdateResult::failure(
                    TransitionBoundaryError{bounds.error()});
            if ((collision.points[1] - collision.points[0])
                    .cross(collision.points[2] - collision.points[0])
                    .squaredNorm() == Scalar{0})
                return UpdateResult::failure(TransitionBoundaryError{
                    SpatialError::DegenerateTriangle});
            if (regular_candidate_geometry_prevalidated_ &&
                triangle.owner.role ==
                    BoundaryOwnerRole::RegularCandidate)
                static_hits.push_back(false);
            else
            {
                const auto hit = static_obstacle_context_->intersects(
                    triangle, nullptr, &obstacle_scratch);
                if (!hit.hasValue())
                    return UpdateResult::failure(hit.error());
                static_hits.push_back(hit.value());
                ++next_boundary.diagnostics_.static_obstacle_queries;
            }
        }

        for (const std::uint32_t id : removed_ids)
        {
            const auto old_key = primitive_keys_.find(id);
            const auto active = old_key == primitive_keys_.end()
                ? active_primitives_.end()
                : active_primitives_.find(old_key->second);
            if (active != active_primitives_.end() &&
                traceFace(active->second.triangle.owner.source_face_id))
                std::cerr << "trace index_remove source="
                          << active->second.triangle.owner.source_face_id
                          << " primitive=" << id << '\n';
            const auto erased = collision_index_->eraseGroup(
                static_cast<CollisionGroupId>(id) + 2);
            if (!erased.hasValue())
                return UpdateResult::failure(
                    TransitionBoundaryError{erased.error()});
            const auto key = primitive_keys_.find(id);
            if (key != primitive_keys_.end())
            {
                active_primitives_.erase(key->second);
                primitive_keys_.erase(key);
            }
        }
        for (const auto id : removed_ids)
        {
            static_hit_ids_.erase(id);
            colliding_primitive_ids_.erase(id);
            const auto neighbors = contact_neighbors_.find(id);
            if (neighbors == contact_neighbors_.end()) continue;
            for (const auto other : neighbors->second)
            {
                contacts_.erase(contactKey(id, other));
                const auto reverse = contact_neighbors_.find(other);
                if (reverse != contact_neighbors_.end())
                {
                    reverse->second.erase(id);
                    if (reverse->second.empty()) contact_neighbors_.erase(reverse);
                }
                refreshCollidingPrimitive(other);
            }
            contact_neighbors_.erase(id);
        }

        std::vector<std::uint32_t> added_ids;
        std::vector<CollisionPrimitiveId> candidate_scratch;
        std::vector<CollisionPrimitiveId> query_contacts;
        for (std::size_t index = 0; index < additions.size(); ++index)
        {
            const OwnedBoundaryTriangle &triangle = additions[index];
            const std::uint32_t id = next_primitive_id_++;
            if (traceFace(triangle.owner.source_face_id))
                std::cerr << "trace index_insert source="
                          << triangle.owner.source_face_id
                          << " primitive=" << id << " role="
                          << roleName(triangle.owner.role)
                          << " layer=" << triangle.owner.layer << '\n';
            auto collision = makeTransitionCollisionTriangle(triangle, id);
            const auto inserted = collision_index_->insertGroup({
                static_cast<CollisionGroupId>(id) + 2, {collision}});
            if (!inserted.hasValue())
                return UpdateResult::failure(
                    TransitionBoundaryError{inserted.error()});
            const TransitionTriangleKey key =
                transitionTriangleKey(triangle);
            active_primitives_[key] =
                {id, triangle, static_hits[index]};
            if (static_hits[index])
            {
                static_hit_ids_.insert(id);
                colliding_primitive_ids_.insert(id);
            }
            primitive_keys_[id] = key;
            added_ids.push_back(id);
        }
        const auto maintained = collision_index_->rebuildIfDegraded();
        if (!maintained.hasValue())
            return UpdateResult::failure(
                TransitionBoundaryError{maintained.error()});
        if (maintained.value())
            ++next_boundary.diagnostics_.transition_index_rebuilds;
        for (const std::uint32_t id : added_ids)
        {
            const auto key = primitive_keys_.find(id);
            const auto active = active_primitives_.find(key->second);
            const CollisionTriangle query = makeTransitionCollisionTriangle(
                active->second.triangle, id);
            collision_index_->queryIllegalContacts(
                query, candidate_scratch, query_contacts,
                static_cast<CollisionGroupId>(id) + 2);
            for (const CollisionPrimitiveId contact : query_contacts)
            {
                const std::uint32_t other =
                    collision_index_->primitive(contact).owner_id;
                if (regular_candidate_geometry_prevalidated_ &&
                    active->second.triangle.owner.role ==
                        BoundaryOwnerRole::RegularCandidate)
                {
                    const auto other_key = primitive_keys_.find(other);
                    const auto other_active = other_key == primitive_keys_.end()
                        ? active_primitives_.end()
                        : active_primitives_.find(other_key->second);
                    if (other_active != active_primitives_.end() &&
                        other_active->second.triangle.owner.role ==
                            BoundaryOwnerRole::RegularCandidate)
                        continue;
                }
                contacts_.insert(contactKey(id, other));
                contact_neighbors_[id].insert(other);
                contact_neighbors_[other].insert(id);
                colliding_primitive_ids_.insert(id);
                colliding_primitive_ids_.insert(other);
            }
            ++next_boundary.diagnostics_.self_collision_queries;
        }
        next_boundary.diagnostics_.self_collision_exact_tests =
            collision_index_->diagnostics().exact_tests;
        next_boundary.diagnostics_.self_candidate_visits =
            collision_index_->diagnostics().candidate_visits;
        next_boundary.diagnostics_.self_duplicate_candidate_visits =
            collision_index_->diagnostics().duplicate_candidate_visits;
        for (const auto &key : affected)
        {
            buckets_.erase(key);
            auto entry = next_boundary.buckets_.extract(key);
            if (!entry.empty()) buckets_.insert(std::move(entry));
        }
        for (const auto &owner : changed)
        {
            owner_keys_.erase(owner);
            auto entry = next_boundary.owner_keys_.extract(owner);
            if (!entry.empty()) owner_keys_.insert(std::move(entry));
        }
        diagonal_requirements_ =
            std::move(next_boundary.diagonal_requirements_);
        exposed_dirty_ = true;
        diagnostics_ = next_boundary.diagnostics_;
        materializeCollisionReport();
        traceWatchedPair("update");
        const auto scanned = std::chrono::steady_clock::now();
        std::cerr << "temporary transition update copy_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(copied-started).count()
            << " boundary_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(replaced-copied).count()
            << " diff_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(diffed-replaced).count()
            << " scan_ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(scanned-diffed).count()
            << " changed=" << changed_owners.size()
            << " added=" << additions.size() << '\n';
        return UpdateResult::success(std::monostate{});
    }

    std::vector<LayerBoundaryOwnerKey>
    IncrementalTransitionCollisionState::ownersForSources(
        const TransitionBoundaryInput &next,
        const std::optional<std::set<SurfaceFaceId>> &sources,
        const std::vector<OwnedBoundaryTriangle> *changed_triangles) const
    {
        std::set<LayerBoundaryOwnerKey> owners;
        if (!sources)
        {
            for (const auto &[owner, keys] : owner_keys_)
            {
                (void)keys;
                owners.insert(owner);
            }
        }
        else
        {
            for (const SurfaceFaceId source : *sources)
            {
                auto owner = owner_keys_.lower_bound({
                    source, 0, BoundaryOwnerRole::RegularCandidate});
                while (owner != owner_keys_.end() &&
                       owner->first.source_face_id == source)
                {
                    owners.insert(owner->first);
                    ++owner;
                }
            }
        }
        const auto &candidates = changed_triangles != nullptr
            ? *changed_triangles : next.candidate_triangles;
        for (const auto &triangle : candidates)
            if (!sources || sources->count(triangle.owner.source_face_id))
                owners.insert(layerBoundaryOwnerKey(triangle.owner));
        return {owners.begin(), owners.end()};
    }

    Result<std::monostate, TransitionBoundaryError>
    IncrementalTransitionCollisionState::initializeCollisions(
        const TransitionBoundaryInput &input)
    {
        regular_candidate_geometry_prevalidated_ =
            input.regular_candidate_geometry_prevalidated;
        original_surface_ = input.original_surface;
        historical_boundary_ = input.historical_boundary;
        historical_index_ = input.historical_index;
        historical_index_includes_transition_ =
            input.historical_index_includes_transition;
        prior_transition_boundary_ = input.prior_transition_boundary;
        sliding_surface_ = input.sliding_surface;
        active_primitives_.clear();
        primitive_keys_.clear();
        contacts_.clear();
        contact_neighbors_.clear();
        static_hit_ids_.clear();
        colliding_primitive_ids_.clear();
        next_primitive_id_ = 0;
        const auto context_started = std::chrono::steady_clock::now();
        auto static_context = TransitionStaticObstacleContext::build(input);
        if (!static_context.hasValue())
            return Result<std::monostate,
                TransitionBoundaryError>::failure(static_context.error());
        static_obstacle_context_ = std::move(static_context.value());
        const auto context_built = std::chrono::steady_clock::now();
        std::vector<CollisionPrimitiveGroup> groups;
        groups.reserve(exposed_boundary_.size());
        const auto obstacle_scan_started =
            std::chrono::steady_clock::now();
        TransitionStaticObstacleQueryScratch obstacle_scratch;
        std::vector<const OwnedBoundaryTriangle *> ordered_triangles;
        ordered_triangles.reserve(exposed_boundary_.size());
        if (regular_candidate_geometry_prevalidated_)
        {
            // Assign lower primitive IDs to transition geometry. Transition
            // queries can then use query-after to test each transition pair
            // once and still see every regular candidate as a later target.
            for (const OwnedBoundaryTriangle &triangle : exposed_boundary_)
                if (triangle.owner.role !=
                    BoundaryOwnerRole::RegularCandidate)
                    ordered_triangles.push_back(&triangle);
            for (const OwnedBoundaryTriangle &triangle : exposed_boundary_)
                if (triangle.owner.role ==
                    BoundaryOwnerRole::RegularCandidate)
                    ordered_triangles.push_back(&triangle);
        }
        else
        {
            for (const OwnedBoundaryTriangle &triangle : exposed_boundary_)
                ordered_triangles.push_back(&triangle);
        }
        for (const OwnedBoundaryTriangle *triangle_ptr : ordered_triangles)
        {
            const OwnedBoundaryTriangle &triangle = *triangle_ptr;
            const std::uint32_t id = next_primitive_id_++;
            if (traceFace(triangle.owner.source_face_id))
                std::cerr << "trace index_insert source="
                          << triangle.owner.source_face_id
                          << " primitive=" << id << " role="
                          << roleName(triangle.owner.role)
                          << " layer=" << triangle.owner.layer
                          << " phase=initial\n";
            groups.push_back({
                static_cast<CollisionGroupId>(id) + 2,
                {makeTransitionCollisionTriangle(triangle, id)}});
            bool static_hit_value = false;
            if (!regular_candidate_geometry_prevalidated_ ||
                triangle.owner.role !=
                    BoundaryOwnerRole::RegularCandidate)
            {
                const auto static_hit = static_obstacle_context_->intersects(
                    triangle, nullptr, &obstacle_scratch);
                if (!static_hit.hasValue())
                    return Result<std::monostate,
                        TransitionBoundaryError>::failure(static_hit.error());
                static_hit_value = static_hit.value();
                ++diagnostics_.static_obstacle_queries;
            }
            const TransitionTriangleKey key =
                transitionTriangleKey(triangle);
            active_primitives_[key] = {id, triangle, static_hit_value};
            if (static_hit_value)
            {
                static_hit_ids_.insert(id);
                colliding_primitive_ids_.insert(id);
            }
            primitive_keys_[id] = key;
        }
        const auto obstacle_scan_done = std::chrono::steady_clock::now();
        auto index = IncrementalCollisionIndex::build(std::move(groups));
        if (!index.hasValue())
            return Result<std::monostate,
                TransitionBoundaryError>::failure(
                    TransitionBoundaryError{index.error()});
        collision_index_ = std::move(index.value());
        const auto index_built = std::chrono::steady_clock::now();
        std::vector<CollisionPrimitiveId> candidate_scratch;
        std::vector<CollisionPrimitiveId> query_contacts;
        for (const auto &[key, active] : active_primitives_)
        {
            (void)key;
            if (regular_candidate_geometry_prevalidated_ &&
                active.triangle.owner.role ==
                    BoundaryOwnerRole::RegularCandidate)
                continue;
            const CollisionTriangle query = makeTransitionCollisionTriangle(
                active.triangle, active.id);
            collision_index_->queryIllegalContactsAfter(
                query, active.id, candidate_scratch, query_contacts,
                static_cast<CollisionGroupId>(active.id) + 2);
            for (const CollisionPrimitiveId contact : query_contacts)
            {
                const std::uint32_t other =
                    collision_index_->primitive(contact).owner_id;
                contacts_.insert(contactKey(active.id, other));
                contact_neighbors_[active.id].insert(other);
                contact_neighbors_[other].insert(active.id);
                colliding_primitive_ids_.insert(active.id);
                colliding_primitive_ids_.insert(other);
            }
            ++diagnostics_.self_collision_queries;
        }
        const auto self_scan_done = std::chrono::steady_clock::now();
        diagnostics_.self_collision_exact_tests =
            collision_index_->diagnostics().exact_tests;
        diagnostics_.self_candidate_visits =
            collision_index_->diagnostics().candidate_visits;
        diagnostics_.self_duplicate_candidate_visits =
            collision_index_->diagnostics().duplicate_candidate_visits;
        diagnostics_.full_collision_builds = 1;
        materializeCollisionReport();
        traceWatchedPair("initial");
        const auto report_done = std::chrono::steady_clock::now();
        const auto milliseconds = [](auto begin, auto end)
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                end - begin).count();
        };
        std::cerr << "temporary transition init context_ms="
                  << milliseconds(context_started, context_built)
                  << " obstacle_ms="
                  << milliseconds(obstacle_scan_started, obstacle_scan_done)
                  << " self_index_build_ms="
                  << milliseconds(obstacle_scan_done, index_built)
                  << " self_query_ms="
                  << milliseconds(index_built, self_scan_done)
                  << " report_ms="
                  << milliseconds(self_scan_done, report_done)
                  << " triangles=" << exposed_boundary_.size() << '\n';
        return Result<std::monostate,
            TransitionBoundaryError>::success(std::monostate{});
    }

    void IncrementalTransitionCollisionState::materializeCollisionReport()
    {
        collision_report_ = {};
        diagnostics_.reported_colliding_primitives =
            colliding_primitive_ids_.size();
        std::set<LayerBoundaryOwnerKey> owners;
        for (const std::uint32_t id : colliding_primitive_ids_)
        {
            const auto key = primitive_keys_.find(id);
            if (key == primitive_keys_.end()) continue;
            const auto active = active_primitives_.find(key->second);
            if (active == active_primitives_.end()) continue;
            const LayerBoundaryOwner &owner = active->second.triangle.owner;
            if (owners.insert(layerBoundaryOwnerKey(owner)).second)
                collision_report_.colliding_owners.push_back(owner);
            appendLayerBoundaryRollbackFaces(
                owner, collision_report_.rollback_faces);
        }
        std::sort(collision_report_.rollback_faces.begin(),
                  collision_report_.rollback_faces.end());
        collision_report_.rollback_faces.erase(std::unique(
            collision_report_.rollback_faces.begin(),
            collision_report_.rollback_faces.end()),
            collision_report_.rollback_faces.end());
    }

    void IncrementalTransitionCollisionState::traceWatchedPair(
        const char *phase)
    {
        constexpr std::array<SurfaceFaceId, 2> watched{{298895, 445377}};
        if (!traceFace(watched[0]) && !traceFace(watched[1])) return;

        std::map<SurfaceFaceId, std::size_t> contributions;
        std::map<SurfaceFaceId, std::size_t> exposed_counts;
        std::map<SurfaceFaceId, std::vector<const ActivePrimitive *>> active;
        for (const auto &[key, values] : buckets_)
        {
            (void)key;
            const bool contains_watched = std::any_of(
                values.begin(), values.end(),
                [](const OwnedBoundaryTriangle &triangle)
                { return traceFace(triangle.owner.source_face_id); });
            for (const auto &triangle : values)
                if (traceFace(triangle.owner.source_face_id))
                    ++contributions[triangle.owner.source_face_id];
            if (values.size() > 1 && contains_watched)
            {
                std::cerr << "trace boundary_bucket phase=" << phase
                          << " contribution_count=" << values.size()
                          << " parity=" << (values.size() % 2)
                          << " owners=";
                for (const auto &triangle : values)
                    std::cerr << triangle.owner.source_face_id << ':'
                              << triangle.owner.layer << ':'
                              << static_cast<int>(triangle.owner.role) << ',';
                std::cerr << '\n';
            }
        }
        for (const auto &triangle : exposedBoundary())
            if (traceFace(triangle.owner.source_face_id))
                ++exposed_counts[triangle.owner.source_face_id];
        for (const auto &[key, primitive] : active_primitives_)
        {
            (void)key;
            if (traceFace(primitive.triangle.owner.source_face_id))
                active[primitive.triangle.owner.source_face_id].push_back(
                    &primitive);
        }

        const auto rollbackContains = [&](SurfaceFaceId id)
        {
            return std::binary_search(
                collision_report_.rollback_faces.begin(),
                collision_report_.rollback_faces.end(), id);
        };
        for (const SurfaceFaceId id : watched)
            if (traceFace(id))
            {
                const auto active_it = active.find(id);
                const std::size_t active_count = active_it == active.end()
                    ? 0 : active_it->second.size();
                const auto exposed_it = exposed_counts.find(id);
                const std::size_t exposed_count = exposed_it == exposed_counts.end()
                    ? 0 : exposed_it->second;
                const auto contribution_it = contributions.find(id);
                const std::size_t contribution_count =
                    contribution_it == contributions.end()
                        ? 0 : contribution_it->second;
                std::size_t static_hits = 0;
                if (active_it != active.end())
                    for (const ActivePrimitive *primitive : active_it->second)
                        static_hits += static_hit_ids_.count(primitive->id);
                std::cerr << "trace pair_state phase=" << phase
                          << " source=" << id
                          << " bucket_contributions=" << contribution_count
                          << " exposed_triangles=" << exposed_count
                          << " indexed_triangles=" << active_count
                          << " static_hit_triangles=" << static_hits
                          << " rollback=" << rollbackContains(id)
                          << '\n';
            }

        for (const auto &[left_id, right_id] : contacts_)
        {
            const auto left_key = primitive_keys_.find(left_id);
            const auto right_key = primitive_keys_.find(right_id);
            if (left_key == primitive_keys_.end() ||
                right_key == primitive_keys_.end()) continue;
            const auto left_active = active_primitives_.find(left_key->second);
            const auto right_active = active_primitives_.find(right_key->second);
            if (left_active == active_primitives_.end() ||
                right_active == active_primitives_.end()) continue;
            const auto &left_owner = left_active->second.triangle.owner;
            const auto &right_owner = right_active->second.triangle.owner;
            if (!traceFace(left_owner.source_face_id) &&
                !traceFace(right_owner.source_face_id)) continue;
            std::cerr << "trace recorded_contact phase=" << phase
                      << " left=" << left_owner.source_face_id << ':'
                      << left_owner.layer << ':' << roleName(left_owner.role)
                      << " right=" << right_owner.source_face_id << ':'
                      << right_owner.layer << ':' << roleName(right_owner.role)
                      << " left_rollback_deps=";
            for (const SurfaceFaceId dep : left_owner.rollback_high_faces)
                std::cerr << dep << ',';
            std::cerr << " right_rollback_deps=";
            for (const SurfaceFaceId dep : right_owner.rollback_high_faces)
                std::cerr << dep << ',';
            std::cerr
                      << " rollback_left="
                      << rollbackContains(left_owner.source_face_id)
                      << " rollback_right="
                      << rollbackContains(right_owner.source_face_id)
                      << '\n';
        }

        const auto left_it = active.find(watched[0]);
        const auto right_it = active.find(watched[1]);
        if (left_it == active.end() || right_it == active.end())
        {
            std::cerr << "trace pair_compare phase=" << phase
                      << " left_indexed=" << (left_it != active.end())
                      << " right_indexed=" << (right_it != active.end())
                      << " direct=unavailable index=unavailable\n";
            return;
        }

        for (const ActivePrimitive *left : left_it->second)
            for (const ActivePrimitive *right : right_it->second)
            {
                const auto left_triangle = makeTransitionCollisionTriangle(
                    left->triangle, left->id);
                const auto right_triangle = makeTransitionCollisionTriangle(
                    right->triangle, right->id);
                const auto exact = hasIllegalTriangleContact(
                    left_triangle, right_triangle);
                std::vector<CollisionPrimitiveId> scratch;
                std::vector<CollisionPrimitiveId> left_contacts;
                std::vector<CollisionPrimitiveId> right_contacts;
                const auto left_bounds = makeAabb(
                    left_triangle.points[0], left_triangle.points[1],
                    left_triangle.points[2]);
                const auto right_bounds = makeAabb(
                    right_triangle.points[0], right_triangle.points[1],
                    right_triangle.points[2]);
                const auto left_candidates = left_bounds.hasValue()
                    ? collision_index_->queryCandidates(
                          left_bounds.value(),
                          static_cast<CollisionGroupId>(left->id) + 2)
                    : std::vector<CollisionPrimitiveId>{};
                const auto right_candidates = right_bounds.hasValue()
                    ? collision_index_->queryCandidates(
                          right_bounds.value(),
                          static_cast<CollisionGroupId>(right->id) + 2)
                    : std::vector<CollisionPrimitiveId>{};
                collision_index_->queryIllegalContacts(
                    left_triangle, scratch, left_contacts,
                    static_cast<CollisionGroupId>(left->id) + 2);
                collision_index_->queryIllegalContacts(
                    right_triangle, scratch, right_contacts,
                    static_cast<CollisionGroupId>(right->id) + 2);
                const auto contains_owner = [&](const auto &contacts,
                                                std::uint32_t owner)
                {
                    return std::any_of(
                        contacts.begin(), contacts.end(),
                        [&](CollisionPrimitiveId contact)
                        {
                            return collision_index_->primitive(contact).owner_id == owner;
                        });
                };
                const bool indexed_hit =
                    contains_owner(left_contacts, right->id) ||
                    contains_owner(right_contacts, left->id);
                const auto contains_candidate_owner = [&](const auto &candidates,
                                                          std::uint32_t owner)
                {
                    return std::any_of(
                        candidates.begin(), candidates.end(),
                        [&](CollisionPrimitiveId candidate)
                        {
                            return collision_index_->primitive(candidate).owner_id == owner;
                        });
                };
                const bool broadphase_hit =
                    contains_candidate_owner(left_candidates, right->id) ||
                    contains_candidate_owner(right_candidates, left->id);
                std::cerr << std::setprecision(17)
                          << "trace pair_compare phase=" << phase
                          << " left_primitive=" << left->id
                          << " left_role=" << roleName(left->triangle.owner.role)
                          << " left_layer=" << left->triangle.owner.layer
                          << " right_primitive=" << right->id
                          << " right_role=" << roleName(right->triangle.owner.role)
                          << " right_layer=" << right->triangle.owner.layer
                          << " direct=" << (exact.hasValue()
                              ? (exact.value() ? "illegal" : "clear") : "error")
                          << " broadphase=" << (broadphase_hit ? "hit" : "miss")
                          << " index_exact=" << (indexed_hit ? "hit" : "miss")
                          << " regular_regular_scan_skip="
                          << (regular_candidate_geometry_prevalidated_ &&
                              left->triangle.owner.role ==
                                  BoundaryOwnerRole::RegularCandidate &&
                              right->triangle.owner.role ==
                                  BoundaryOwnerRole::RegularCandidate)
                          << " left_keys=";
                for (const auto &key : left->triangle.vertex_keys)
                    std::cerr << key.source_vertex_id << ':' << key.layer
                              << ':' << key.branch_id << ',';
                std::cerr << " right_keys=";
                for (const auto &key : right->triangle.vertex_keys)
                    std::cerr << key.source_vertex_id << ':' << key.layer
                              << ':' << key.branch_id << ',';
                std::cerr << " left_points=";
                for (const auto &point : left->triangle.points)
                    std::cerr << '(' << point.x() << ',' << point.y() << ','
                              << point.z() << ')';
                std::cerr << " right_points=";
                for (const auto &point : right->triangle.points)
                    std::cerr << '(' << point.x() << ',' << point.y() << ','
                              << point.z() << ')';
                std::cerr << '\n';
            }
    }

    TransitionCollisionFailureDiagnostics
    IncrementalTransitionCollisionState::failureDiagnostics(
        std::size_t sample_limit) const
    {
        TransitionCollisionFailureDiagnostics result;
        std::map<LayerBoundaryOwnerKey, LayerBoundaryOwner> owners;
        std::set<LayerBoundaryOwnerKey> static_owners;
        std::set<LayerBoundaryOwnerKey> self_owners;
        const auto ownerForPrimitive = [&](std::uint32_t id)
            -> const LayerBoundaryOwner *
        {
            const auto key = primitive_keys_.find(id);
            if (key == primitive_keys_.end()) return nullptr;
            const auto active = active_primitives_.find(key->second);
            return active == active_primitives_.end()
                ? nullptr : &active->second.triangle.owner;
        };
        for (const std::uint32_t id : colliding_primitive_ids_)
        {
            const LayerBoundaryOwner *owner = ownerForPrimitive(id);
            if (owner == nullptr) continue;
            const LayerBoundaryOwnerKey key = layerBoundaryOwnerKey(*owner);
            owners.emplace(key, *owner);
            if (static_hit_ids_.find(id) != static_hit_ids_.end())
                static_owners.insert(key);
            const auto neighbors = contact_neighbors_.find(id);
            if (neighbors != contact_neighbors_.end() &&
                !neighbors->second.empty())
                self_owners.insert(key);
        }
        for (const auto &[key, owner] : owners)
        {
            const auto role = static_cast<std::size_t>(owner.role);
            if (role < result.owners_by_role.size())
                ++result.owners_by_role[role];
            if (result.owner_samples.size() < sample_limit)
                result.owner_samples.push_back(owner);
        }
        result.owners_with_static_contacts = static_owners.size();
        result.owners_with_self_contacts = self_owners.size();

        std::set<std::pair<LayerBoundaryOwnerKey, LayerBoundaryOwnerKey>>
            seen_pairs;
        for (const auto &[left_id, right_id] : contacts_)
        {
            const LayerBoundaryOwner *left = ownerForPrimitive(left_id);
            const LayerBoundaryOwner *right = ownerForPrimitive(right_id);
            if (left == nullptr || right == nullptr) continue;
            auto left_key = layerBoundaryOwnerKey(*left);
            auto right_key = layerBoundaryOwnerKey(*right);
            if (left_key == right_key) continue;
            if (right_key < left_key)
            {
                std::swap(left_key, right_key);
                std::swap(left, right);
            }
            if (seen_pairs.emplace(left_key, right_key).second)
            {
                ++result.self_contact_owner_pairs;
                if (result.self_contact_samples.size() < sample_limit)
                    result.self_contact_samples.emplace_back(*left, *right);
            }
        }
        return result;
    }

    void IncrementalTransitionCollisionState::refreshCollidingPrimitive(
        std::uint32_t id)
    {
        const auto neighbors = contact_neighbors_.find(id);
        if (static_hit_ids_.find(id) != static_hit_ids_.end() ||
            (neighbors != contact_neighbors_.end() &&
             !neighbors->second.empty()))
            colliding_primitive_ids_.insert(id);
        else
            colliding_primitive_ids_.erase(id);
    }
}
