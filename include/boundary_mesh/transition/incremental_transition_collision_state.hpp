#pragma once

#include <map>
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

        UpdateResult replaceBoundary(
            const TransitionBoundaryInput &next,
            const std::vector<LayerBoundaryOwnerKey> &changed_owners);

        const std::vector<OwnedBoundaryTriangle> &exposedBoundary() const
        { return exposed_boundary_; }

        const IncrementalTransitionCollisionDiagnostics &diagnostics() const
        { return diagnostics_; }

    private:
        using BucketMap = std::map<
            TransitionTriangleKey,
            std::vector<OwnedBoundaryTriangle>>;
        using OwnerKeyMap = std::map<
            LayerBoundaryOwnerKey,
            std::set<TransitionTriangleKey>>;

        static Result<std::monostate, TransitionBoundaryError>
        validateDiagonals(const TransitionBoundaryInput &input);
        void materializeExposed();

        BucketMap buckets_;
        OwnerKeyMap owner_keys_;
        std::vector<LayerDiagonalRequirement> diagonal_requirements_;
        std::vector<OwnedBoundaryTriangle> exposed_boundary_;
        IncrementalTransitionCollisionDiagnostics diagnostics_;
    };
}
