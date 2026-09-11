#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
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

    struct LayerDiagonalRequirement
    {
        LayerQuadFaceKey key;
        QuadDiagonal diagonal{};
    };

    struct TransitionBoundaryInput
    {
        std::vector<OwnedBoundaryTriangle> candidate_triangles;
        std::vector<LayerDiagonalRequirement> diagonal_requirements;
        const CollisionIndex *original_surface{};
        const ExposedBoundaryTracker *historical_boundary{};
        const std::vector<OwnedBoundaryTriangle>
            *prior_transition_boundary{};
        const SlidingIntersectionIndex *sliding_surface{};
    };

    using TransitionBoundaryError = std::variant<
        SpatialError,
        ConflictingLayerQuadDiagonal>;

    class TransitionBoundaryChecker
    {
    public:
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
