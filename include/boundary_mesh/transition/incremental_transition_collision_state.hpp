#pragma once

#include <array>
#include <map>
#include <optional>
#include <set>
#include <variant>

#include <boundary_mesh/transition/transition_boundary_checker.hpp>

namespace boundary_mesh
{
    struct TransitionCollisionFailureDiagnostics
    {
        std::array<std::size_t, 4> owners_by_role{};
        std::size_t owners_with_static_contacts{};
        std::size_t owners_with_self_contacts{};
        std::size_t self_contact_owner_pairs{};
        std::vector<LayerBoundaryOwner> owner_samples;
        std::vector<std::pair<LayerBoundaryOwner, LayerBoundaryOwner>>
            self_contact_samples;
    };

    struct IncrementalTransitionCollisionDiagnostics
    {
        std::size_t changed_owners{};
        std::size_t affected_triangle_keys{};
        std::size_t staged_exposed_triangles{};
        std::size_t exposed_inserts{};
        std::size_t exposed_erases{};
        std::size_t exposed_replacements{};
        std::size_t static_obstacle_queries{};
        std::size_t self_collision_queries{};
        std::size_t self_collision_exact_tests{};
        std::size_t self_candidate_visits{};
        std::size_t self_duplicate_candidate_visits{};
        std::size_t transition_index_rebuilds{};
        std::size_t reported_colliding_primitives{};
        std::size_t full_collision_builds{};
    };

    class IncrementalTransitionCollisionState
    {
    public:
        using BuildResult = Result<
            IncrementalTransitionCollisionState,
            TransitionBoundaryError>;
        using UpdateResult = Result<
            std::monostate,
            TransitionBoundaryError>;

        static BuildResult buildBoundary(
            const TransitionBoundaryInput &input);

        static BuildResult build(
            const TransitionBoundaryInput &input);

        UpdateResult replaceBoundary(
            const TransitionBoundaryInput &next,
            const std::vector<LayerBoundaryOwnerKey> &changed_owners);

        UpdateResult update(
            const TransitionBoundaryInput &next,
            const std::vector<LayerBoundaryOwnerKey> &changed_owners);

        std::vector<LayerBoundaryOwnerKey> ownersForSources(
            const TransitionBoundaryInput &next,
            const std::optional<std::set<SurfaceFaceId>> &sources,
            const std::vector<OwnedBoundaryTriangle> *changed_triangles =
                nullptr) const;

        UpdateResult update(
            const TransitionBoundaryInput &next,
            const std::vector<LayerBoundaryOwnerKey> &changed_owners,
            const std::vector<OwnedBoundaryTriangle> &changed_triangles);

        const std::vector<OwnedBoundaryTriangle> &exposedBoundary() const
        {
            if (exposed_dirty_)
            {
                exposed_boundary_.clear();
                for (const auto &[key, contributions] : buckets_)
                    if (contributions.size() % 2 == 1)
                        exposed_boundary_.push_back(contributions.front());
                exposed_dirty_ = false;
            }
            return exposed_boundary_;
        }

        const IncrementalTransitionCollisionDiagnostics &diagnostics() const
        { return diagnostics_; }

        const TransitionCollisionReport &collisionReport() const
        { return collision_report_; }

        TransitionCollisionFailureDiagnostics failureDiagnostics(
            std::size_t sample_limit = 32) const;

    private:
        using BucketMap = std::map<
            TransitionTriangleKey,
            std::vector<OwnedBoundaryTriangle>>;
        using OwnerKeyMap = std::map<
            LayerBoundaryOwnerKey,
            std::set<TransitionTriangleKey>>;
        struct ActivePrimitive
        {
            std::uint32_t id{};
            OwnedBoundaryTriangle triangle;
            bool static_obstacle_hit{};
        };

        static Result<std::monostate, TransitionBoundaryError>
        validateDiagonals(const TransitionBoundaryInput &input);
        UpdateResult replaceBoundaryInPlace(
            const TransitionBoundaryInput &next,
            const std::vector<LayerBoundaryOwnerKey> &changed_owners,
            const std::vector<OwnedBoundaryTriangle> &changed_triangles);
        void materializeExposed();
        Result<std::monostate, TransitionBoundaryError>
        initializeCollisions(const TransitionBoundaryInput &input);
        void materializeCollisionReport();
        void traceWatchedPair(const char *phase);
        void refreshCollidingPrimitive(std::uint32_t id);

        BucketMap buckets_;
        OwnerKeyMap owner_keys_;
        std::vector<LayerDiagonalRequirement> diagonal_requirements_;
        mutable std::vector<OwnedBoundaryTriangle> exposed_boundary_;
        mutable bool exposed_dirty_{};
        IncrementalTransitionCollisionDiagnostics diagnostics_;
        TransitionCollisionReport collision_report_;
        std::map<TransitionTriangleKey, ActivePrimitive> active_primitives_;
        std::map<std::uint32_t, TransitionTriangleKey> primitive_keys_;
        std::set<std::pair<std::uint32_t, std::uint32_t>> contacts_;
        std::map<std::uint32_t, std::set<std::uint32_t>> contact_neighbors_;
        std::set<std::uint32_t> static_hit_ids_;
        std::set<std::uint32_t> colliding_primitive_ids_;
        std::optional<IncrementalCollisionIndex> collision_index_;
        std::optional<TransitionStaticObstacleContext>
            static_obstacle_context_;
        std::uint32_t next_primitive_id_{};
        const CollisionIndex *original_surface_{};
        const ExposedBoundaryTracker *historical_boundary_{};
        const IncrementalCollisionIndex *historical_index_{};
        bool historical_index_includes_transition_{};
        const std::vector<OwnedBoundaryTriangle> *prior_transition_boundary_{};
        const SlidingIntersectionIndex *sliding_surface_{};
        bool regular_candidate_geometry_prevalidated_{};
    };
}
