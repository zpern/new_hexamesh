#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/incremental_collision_index.hpp>
#include <boundary_mesh/spatial/sliding_intersection_index.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>

namespace boundary_mesh
{
    enum class BoundaryOwnerRole
    {
        RegularCandidate,
        TopCap,
        SideTransition,
        ExternalPatch
    };

    struct LayerBoundaryOwner
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        BoundaryOwnerRole role{};
        std::vector<SurfaceFaceId> rollback_high_faces;
    };

    struct LayerBoundaryOwnerKey
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        BoundaryOwnerRole role{};
    };

    struct UnresolvedTransitionCollision
    {
        std::vector<LayerBoundaryOwnerKey> owners;
    };

    inline bool operator==(
        const LayerBoundaryOwnerKey &left,
        const LayerBoundaryOwnerKey &right)
    {
        return left.source_face_id == right.source_face_id &&
               left.layer == right.layer && left.role == right.role;
    }

    inline bool operator<(
        const LayerBoundaryOwnerKey &left,
        const LayerBoundaryOwnerKey &right)
    {
        return std::tie(left.source_face_id, left.layer, left.role) <
               std::tie(right.source_face_id, right.layer, right.role);
    }

    inline bool operator!=(
        const LayerBoundaryOwnerKey &left,
        const LayerBoundaryOwnerKey &right)
    {
        return !(left == right);
    }

    inline void appendLayerBoundaryRollbackFaces(
        const LayerBoundaryOwner &owner,
        std::vector<SurfaceFaceId> &rollback_faces)
    {
        if (!owner.rollback_high_faces.empty())
            rollback_faces.insert(
                rollback_faces.end(),
                owner.rollback_high_faces.begin(),
                owner.rollback_high_faces.end());
        else if (owner.role == BoundaryOwnerRole::RegularCandidate)
            rollback_faces.push_back(owner.source_face_id);
    }

    using TransitionVertexTuple =
        std::tuple<VertexId, std::uint32_t, std::uint32_t>;
    using TransitionTriangleKey =
        std::array<TransitionVertexTuple, 3>;

    struct SlidingColumnContext
    {
        std::vector<Point3> low_points;
        std::vector<Point3> high_points;
        std::vector<std::vector<std::uint32_t>> low_region_ids;
        std::vector<std::vector<std::uint32_t>> high_region_ids;
    };

    struct OwnedBoundaryTriangle
    {
        std::array<Point3, 3> points{};
        std::array<CollisionVertexKey, 3> vertex_keys{};
        LayerBoundaryOwner owner;
        std::array<std::vector<std::uint32_t>, 3>
            vertex_sliding_region_ids;
        std::uint8_t physical_edge_mask{};
        std::vector<std::uint32_t> complete_face_exemption_regions;
        std::shared_ptr<const SlidingColumnContext> sliding_columns;
    };

    LayerBoundaryOwnerKey layerBoundaryOwnerKey(
        const LayerBoundaryOwner &owner);

    TransitionTriangleKey transitionTriangleKey(
        const OwnedBoundaryTriangle &triangle);

    struct LayerDiagonalRequirement
    {
        LayerQuadFaceKey key;
        QuadDiagonal diagonal{};
    };

    struct TransitionBoundaryInput
    {
        std::vector<OwnedBoundaryTriangle> candidate_triangles;
        // True only when the rule-layer pipeline has already checked all
        // RegularCandidate geometry against static obstacles and itself.
        // Resolver collision state still indexes those triangles so
        // transition geometry can query them.
        bool regular_candidate_geometry_prevalidated{};
        std::vector<LayerDiagonalRequirement> diagonal_requirements;
        const CollisionIndex *original_surface{};
        const ExposedBoundaryTracker *historical_boundary{};
        const IncrementalCollisionIndex *historical_index{};
        // historical_index contains both regular exposed faces and committed
        // transition faces when this flag is set.
        bool historical_index_includes_transition{};
        const std::vector<OwnedBoundaryTriangle>
            *prior_transition_boundary{};
        const CollisionIndex *prior_transition_index{};
        const IncrementalCollisionIndex *prior_transition_dynamic_index{};
        const SlidingIntersectionIndex *sliding_surface{};
    };

    using TransitionBoundaryError = std::variant<
        SpatialError,
        ConflictingLayerQuadDiagonal,
        UnresolvedTransitionCollision>;

    struct TransitionCollisionReport
    {
        std::vector<LayerBoundaryOwner> colliding_owners;
        std::vector<SurfaceFaceId> rollback_faces;
    };

    struct TransitionStaticObstacleQueryScratch
    {
        std::vector<std::size_t> candidate_ids;
        std::vector<std::size_t> traversal_nodes;
        std::vector<std::size_t> contact_ids;
        std::vector<std::uint64_t> incremental_candidate_ids;
        std::vector<std::uint64_t> incremental_contact_ids;
    };

    class TransitionStaticObstacleContext
    {
    public:
        using BuildResult = Result<TransitionStaticObstacleContext,
            TransitionBoundaryError>;

        static BuildResult build(const TransitionBoundaryInput &input);

        Result<bool, TransitionBoundaryError> intersects(
            const OwnedBoundaryTriangle &triangle,
            std::vector<TrianglePoints> *collided_faces = nullptr,
            TransitionStaticObstacleQueryScratch *scratch = nullptr) const;

    private:
        const CollisionIndex *original_surface_{};
        const ExposedBoundaryTracker *historical_boundary_{};
        const IncrementalCollisionIndex *historical_index_{};
        bool historical_index_includes_transition_{};
        const std::vector<OwnedBoundaryTriangle>
            *prior_transition_boundary_{};
        const SlidingIntersectionIndex *sliding_surface_{};
        std::optional<CollisionIndex> immutable_historical_index_;
        const CollisionIndex *shared_prior_transition_index_{};
        std::optional<CollisionIndex> prior_transition_index_;
        const IncrementalCollisionIndex *shared_prior_transition_dynamic_index_{};
    };

    CollisionTriangle makeTransitionCollisionTriangle(
        const OwnedBoundaryTriangle &owned,
        std::uint32_t owner_id);

    class TransitionBoundaryChecker
    {
    public:
        Result<TransitionCollisionReport, TransitionBoundaryError>
        inspect(const TransitionBoundaryInput &input) const;

        Result<std::vector<OwnedBoundaryTriangle>, TransitionBoundaryError>
        assembleExposedBoundary(const TransitionBoundaryInput &input) const;

        Result<std::vector<LayerBoundaryOwner>, TransitionBoundaryError>
        findCollidingOwners(const TransitionBoundaryInput &input) const;

        Result<std::vector<SurfaceFaceId>, TransitionBoundaryError>
        findRollbackFaces(const TransitionBoundaryInput &input) const;

    private:
        Result<std::vector<SurfaceFaceId>, TransitionBoundaryError>
        findRollbackFacesImpl(
            const TransitionBoundaryInput &input,
            std::vector<LayerBoundaryOwner> *colliding_owners) const;
    };
}
