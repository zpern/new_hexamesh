#pragma once

#include <map>
#include <optional>
#include <set>
#include <variant>

#include <boundary_mesh/transition/transition_boundary_checker.hpp>

namespace boundary_mesh
{
    struct IncrementalTransitionCollisionDiagnostics
    {
        std::size_t changed_owners{};
        std::size_t affected_triangle_keys{};
        std::size_t exposed_inserts{};
        std::size_t exposed_erases{};
        std::size_t exposed_replacements{};
        std::size_t static_obstacle_queries{};
        std::size_t self_collision_queries{};
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

        const std::vector<OwnedBoundaryTriangle> &exposedBoundary() const
        { return exposed_boundary_; }

        const IncrementalTransitionCollisionDiagnostics &diagnostics() const
        { return diagnostics_; }

        const TransitionCollisionReport &collisionReport() const
        { return collision_report_; }

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
        void materializeExposed();
        Result<std::monostate, TransitionBoundaryError>
        initializeCollisions(const TransitionBoundaryInput &input);
        void materializeCollisionReport();

        BucketMap buckets_;
        OwnerKeyMap owner_keys_;
        std::vector<LayerDiagonalRequirement> diagonal_requirements_;
        std::vector<OwnedBoundaryTriangle> exposed_boundary_;
        IncrementalTransitionCollisionDiagnostics diagnostics_;
        TransitionCollisionReport collision_report_;
        std::map<TransitionTriangleKey, ActivePrimitive> active_primitives_;
        std::map<std::uint32_t, TransitionTriangleKey> primitive_keys_;
        std::set<std::pair<std::uint32_t, std::uint32_t>> contacts_;
        std::optional<IncrementalCollisionIndex> collision_index_;
        std::optional<TransitionStaticObstacleContext>
            static_obstacle_context_;
        std::uint32_t next_primitive_id_{};
        const CollisionIndex *original_surface_{};
        const ExposedBoundaryTracker *historical_boundary_{};
        const IncrementalCollisionIndex *historical_index_{};
        const std::vector<OwnedBoundaryTriangle> *prior_transition_boundary_{};
        const SlidingIntersectionIndex *sliding_surface_{};
    };
}
