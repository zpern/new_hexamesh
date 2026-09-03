#pragma once

#include <array>
#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>

namespace boundary_mesh
{
    enum class BoundaryOwnerRole
    {
        RegularCandidate,
        TopCap,
        SideTransition
    };

    struct LayerBoundaryOwner
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        BoundaryOwnerRole role{};
        std::vector<SurfaceFaceId> rollback_high_faces;
    };

    struct OwnedBoundaryTriangle
    {
        std::array<Point3, 3> points{};
        std::array<CollisionVertexKey, 3> vertex_keys{};
        LayerBoundaryOwner owner;
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
    };

    using TransitionBoundaryError = std::variant<
        SpatialError,
        ConflictingLayerQuadDiagonal>;

    class TransitionBoundaryChecker
    {
    public:
        Result<std::vector<OwnedBoundaryTriangle>, TransitionBoundaryError>
        assembleExposedBoundary(const TransitionBoundaryInput &input) const;

        Result<std::vector<SurfaceFaceId>, TransitionBoundaryError>
        findRollbackFaces(const TransitionBoundaryInput &input) const;
    };
}
