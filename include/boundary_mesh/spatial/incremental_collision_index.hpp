#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <map>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/spatial/aabb.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>

namespace boundary_mesh
{
    using CollisionPrimitiveId = std::uint64_t;
    using CollisionGroupId = std::uint64_t;

    struct CollisionPrimitiveGroup
    {
        CollisionGroupId id{};
        std::vector<CollisionTriangle> triangles;
    };

    struct IncrementalCollisionIndexOptions
    {
        std::size_t maximum_depth{12};
        std::size_t target_leaf_capacity{64};
        std::size_t rebuild_leaf_capacity{1024};
        Scalar rebuild_inactive_ratio{Scalar{0.30}};
        Scalar root_padding{Scalar{0.05}};
    };

    struct CollisionIndexDiagnostics
    {
        std::uint64_t inserts{}, erases{}, queries{};
        std::uint64_t broad_phase_candidates{}, exact_tests{};
        std::uint64_t rebuilds{}, root_expansions{};
        std::size_t active_primitives{}, inactive_primitives{};
        std::size_t maximum_leaf_load{};
    };

    class IncrementalCollisionIndex
    {
    public:
        static Result<IncrementalCollisionIndex, SpatialError> build(
            std::vector<CollisionPrimitiveGroup> groups,
            IncrementalCollisionIndexOptions options = {});
        Result<std::monostate, SpatialError> insertGroup(
            CollisionPrimitiveGroup group);
        Result<std::monostate, SpatialError> eraseGroup(
            CollisionGroupId group);
        std::vector<CollisionPrimitiveId> queryCandidates(
            const Aabb &bounds,
            std::optional<CollisionGroupId> ignored_group = {}) const;
        std::vector<CollisionPrimitiveId> queryIllegalContacts(
            const CollisionTriangle &triangle,
            std::optional<CollisionGroupId> ignored_group = {}) const;
        const CollisionTriangle &primitive(CollisionPrimitiveId id) const;
        const CollisionIndexDiagnostics &diagnostics() const noexcept;
        Result<bool, SpatialError> rebuildIfDegraded();

    private:
        struct StoredPrimitive
        {
            CollisionTriangle triangle;
            Aabb bounds;
            CollisionGroupId group{};
            bool active{};
        };
        struct Node
        {
            Aabb bounds;
            std::array<std::size_t, 8> children{};
            std::vector<CollisionPrimitiveId> primitives;
            bool leaf{true};
        };

        void rebuildTree();
        std::size_t appendNode(
            const Aabb &bounds,
            const std::vector<CollisionPrimitiveId> &ids,
            std::size_t depth);
        void insertIntoNode(
            std::size_t node,
            CollisionPrimitiveId id);
        void queryNode(
            std::size_t node,
            const Aabb &bounds,
            std::vector<CollisionPrimitiveId> &result) const;
        bool rootContains(const Aabb &bounds) const;

        IncrementalCollisionIndexOptions options_;
        std::vector<StoredPrimitive> primitives_;
        std::map<CollisionGroupId, std::vector<CollisionPrimitiveId>> groups_;
        std::vector<Node> nodes_;
        mutable CollisionIndexDiagnostics diagnostics_;
    };
}
