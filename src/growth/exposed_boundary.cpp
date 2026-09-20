#include <boundary_mesh/growth/exposed_boundary.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <utility>

#include <Eigen/Geometry>

namespace boundary_mesh
{
    namespace
    {
        bool keyLess(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right)
        {
            return left.source_vertex_id < right.source_vertex_id ||
                   (left.source_vertex_id == right.source_vertex_id &&
                    (left.layer < right.layer ||
                     (left.layer == right.layer &&
                      left.branch_id < right.branch_id)));
        }

        bool keyEqual(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right)
        {
            return left.source_vertex_id == right.source_vertex_id &&
                   left.layer == right.layer &&
                   left.branch_id == right.branch_id;
        }

        bool faceKeyLess(
            const BoundaryFaceKey &left,
            const BoundaryFaceKey &right)
        {
            if (left.vertex_count != right.vertex_count)
            {
                return left.vertex_count < right.vertex_count;
            }
            for (std::size_t index = 0; index < left.vertex_count; ++index)
            {
                if (keyLess(left.vertices[index], right.vertices[index]))
                {
                    return true;
                }
                if (keyLess(right.vertices[index], left.vertices[index]))
                {
                    return false;
                }
            }
            return false;
        }

        bool faceKeyEqual(
            const BoundaryFaceKey &left,
            const BoundaryFaceKey &right)
        {
            if (left.vertex_count != right.vertex_count)
            {
                return false;
            }
            for (std::size_t index = 0; index < left.vertex_count; ++index)
            {
                if (!keyEqual(left.vertices[index], right.vertices[index]))
                {
                    return false;
                }
            }
            return true;
        }

        using BoundaryFaceMap = std::map<
            BoundaryFaceKey,
            BoundaryFace,
            BoundaryFaceKeyLess>;

        bool validFace(const BoundaryFace &face)
        {
            if (face.points.size() != face.vertex_keys.size() ||
                (face.points.size() != 3 && face.points.size() != 4))
            {
                return false;
            }
            return std::all_of(
                face.points.begin(),
                face.points.end(),
                [](const Point3 &point) { return point.allFinite(); });
        }

        std::size_t findFace(
            const std::vector<BoundaryFace> &faces,
            const BoundaryFaceKey &key)
        {
            for (std::size_t index = 0; index < faces.size(); ++index)
            {
                const auto candidate_key = makeBoundaryFaceKey(faces[index]);
                if (candidate_key.hasValue() &&
                    faceKeyEqual(candidate_key.value(), key))
                {
                    return index;
                }
            }
            return faces.size();
        }

        void toggleFace(
            BoundaryFaceMap &faces,
            const BoundaryFace &face)
        {
            const BoundaryFaceKey key = makeBoundaryFaceKey(face).value();
            const auto found = faces.find(key);
            if (found == faces.end())
            {
                faces.emplace(key, face);
            }
            else
            {
                faces.erase(found);
            }
        }

        std::vector<BoundaryFace> candidateFaces(
            const LayerBoundaryCandidate &candidate)
        {
            std::vector<BoundaryFace> faces;
            faces.push_back(candidate.top);
            const std::size_t count = candidate.bottom.points.size();
            for (std::size_t index = 0; index < count; ++index)
            {
                const std::size_t next = (index + 1) % count;
                faces.push_back(BoundaryFace{
                    {candidate.bottom.points[index],
                     candidate.bottom.points[next],
                     candidate.top.points[next],
                     candidate.top.points[index]},
                    {candidate.bottom.vertex_keys[index],
                     candidate.bottom.vertex_keys[next],
                     candidate.top.vertex_keys[next],
                     candidate.top.vertex_keys[index]},
                    candidate.top.source_face_id,
                    index < candidate.side_tags.size()
                        ? candidate.side_tags[index].region_id
                        : candidate.top.region_id,
                    index < candidate.side_tags.size()
                        ? candidate.side_tags[index].kind
                        : candidate.top.boundary_kind});
            }
            return faces;
        }

        BoundaryFaceMap makeFaceMap(
            const std::vector<BoundaryFace> &faces)
        {
            BoundaryFaceMap result;
            for (const BoundaryFace &face : faces)
            {
                result.emplace(makeBoundaryFaceKey(face).value(), face);
            }
            return result;
        }

        std::vector<BoundaryFace> mapFaces(
            const BoundaryFaceMap &faces)
        {
            std::vector<BoundaryFace> result;
            result.reserve(faces.size());
            for (const auto &entry : faces)
            {
                result.push_back(entry.second);
            }
            return result;
        }

        Result<std::vector<CollisionTriangle>, SpatialError>
        collisionTrianglesForFace(
            const BoundaryFace &face,
            std::uint32_t owner_id)
        {
            std::vector<CollisionTriangle> triangles;
            const std::array<std::array<std::size_t, 3>, 2> splits{{
                {{0, 1, 2}}, {{0, 2, 3}}}};
            const std::size_t split_count = face.points.size() == 3 ? 1 : 2;
            for (std::size_t split = 0; split < split_count; ++split)
            {
                CollisionTriangle triangle;
                triangle.owner_kind = CollisionOwnerKind::ExposedBoundary;
                triangle.owner_id = owner_id;
                triangle.boundary_vertex_count =
                    static_cast<std::uint8_t>(face.points.size());
                for (std::size_t boundary = 0;
                     boundary < face.points.size(); ++boundary)
                {
                    triangle.boundary_points[boundary] = face.points[boundary];
                    triangle.boundary_vertex_keys[boundary] =
                        face.vertex_keys[boundary];
                }
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const std::size_t index = splits[split][corner];
                    triangle.points[corner] = face.points[index];
                    triangle.vertex_keys[corner] = face.vertex_keys[index];
                }
                if ((triangle.points[1] - triangle.points[0])
                        .cross(triangle.points[2] - triangle.points[0])
                        .squaredNorm() == Scalar{0})
                    return Result<std::vector<CollisionTriangle>, SpatialError>::failure(
                        SpatialError::DegenerateTriangle);
                triangles.push_back(std::move(triangle));
            }
            return Result<std::vector<CollisionTriangle>, SpatialError>::success(
                std::move(triangles));
        }
    }

    ExposedBoundaryTracker::ExposedBoundaryTracker()
        : collision_index_(
              std::move(IncrementalCollisionIndex::build({}).value()))
    {
    }

    bool BoundaryFaceKeyLess::operator()(
        const BoundaryFaceKey &left,
        const BoundaryFaceKey &right) const noexcept
    {
        return faceKeyLess(left, right);
    }

    Result<BoundaryFaceKey, SpatialError> makeBoundaryFaceKey(
        const BoundaryFace &face)
    {
        if (!validFace(face))
        {
            const bool finite = std::all_of(
                face.points.begin(),
                face.points.end(),
                [](const Point3 &point) { return point.allFinite(); });
            return Result<BoundaryFaceKey, SpatialError>::failure(
                finite
                    ? SpatialError::InvalidTopologyReference
                    : SpatialError::NonFiniteCoordinate);
        }
        BoundaryFaceKey key;
        key.vertex_count = static_cast<std::uint8_t>(face.vertex_keys.size());
        std::copy(
            face.vertex_keys.begin(),
            face.vertex_keys.end(),
            key.vertices.begin());
        std::sort(
            key.vertices.begin(),
            key.vertices.begin() + key.vertex_count,
            keyLess);
        for (std::size_t index = 1; index < key.vertex_count; ++index)
        {
            if (keyEqual(key.vertices[index - 1], key.vertices[index]))
            {
                return Result<BoundaryFaceKey, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            }
        }
        return Result<BoundaryFaceKey, SpatialError>::success(key);
    }

    Result<ExposedBoundaryUpdate, SpatialError>
    ExposedBoundaryTracker::prepare(
        const std::vector<LayerBoundaryCandidate> &candidates) const
    {
        const BoundaryFaceMap original = makeFaceMap(faces_);
        BoundaryFaceMap working = original;
        for (const LayerBoundaryCandidate &candidate : candidates)
        {
            if (!validFace(candidate.bottom) ||
                !validFace(candidate.top) ||
                candidate.bottom.points.size() != candidate.top.points.size())
            {
                return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            }
            const auto bottom_key = makeBoundaryFaceKey(candidate.bottom);
            const auto top_key = makeBoundaryFaceKey(candidate.top);
            if (!bottom_key.hasValue() || !top_key.hasValue())
            {
                return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                    !bottom_key.hasValue()
                        ? bottom_key.error()
                        : top_key.error());
            }
            working.erase(bottom_key.value());
            for (const BoundaryFace &face : candidateFaces(candidate))
            {
                const auto key = makeBoundaryFaceKey(face);
                if (!key.hasValue())
                {
                    return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                        key.error());
                }
                toggleFace(working, face);
            }
        }
        ExposedBoundaryUpdate update;
        for (const auto &entry : original)
        {
            if (working.find(entry.first) == working.end())
            {
                update.erase_faces.push_back(entry.first);
            }
        }
        for (const auto &entry : working)
        {
            if (original.find(entry.first) == original.end())
            {
                update.insert_faces.push_back(entry.second);
            }
        }
        return Result<ExposedBoundaryUpdate, SpatialError>::success(
            std::move(update));
    }

    Result<std::monostate, SpatialError> ExposedBoundaryTracker::apply(
        const ExposedBoundaryUpdate &update)
    {
        const auto preparation_start = std::chrono::steady_clock::now();
        struct PendingInsert
        {
            BoundaryFaceKey key;
            BoundaryFace face;
            std::vector<CollisionTriangle> triangles;
            bool indexed{};
        };
        std::vector<PendingInsert> pending;
        BoundaryFaceMap working = makeFaceMap(faces_);
        auto prospective_groups = collision_groups_;

        for (const BoundaryFaceKey &key : update.erase_faces)
        {
            if (prospective_groups.erase(key) != 1 || working.erase(key) != 1)
                return Result<std::monostate, SpatialError>::failure(
                    SpatialError::MissingPrimitiveGroup);
        }

        for (const BoundaryFace &face : update.insert_faces)
        {
            const auto key = makeBoundaryFaceKey(face);
            if (!key.hasValue())
                return Result<std::monostate, SpatialError>::failure(key.error());
            const bool indexed = CollisionBoundaryPolicy{}.isObstacle(
                face.boundary_kind,
                CollisionSurfaceOrigin::GeneratedBoundary);
            std::vector<CollisionTriangle> triangles;
            if (indexed)
            {
                const auto built = collisionTrianglesForFace(
                    face, face.source_face_id);
                if (!built.hasValue())
                    return Result<std::monostate, SpatialError>::failure(
                        built.error());
                triangles = built.value();
            }
            if (prospective_groups.find(key.value()) != prospective_groups.end() ||
                working.find(key.value()) != working.end())
                return Result<std::monostate, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            prospective_groups.emplace(key.value(), CollisionGroupId{});
            working.emplace(key.value(), face);
            pending.push_back(
                {key.value(), face, std::move(triangles), indexed});
        }

        const std::size_t changed_faces = std::max(
            update.erase_faces.size(), update.insert_faces.size());
        const bool bulk_rebuild = faces_.empty() ||
            changed_faces * 5 >= faces_.size();
        const auto preparation_end = std::chrono::steady_clock::now();
        const auto index_start = preparation_end;

        IncrementalCollisionIndex replacement_index = collision_index_;
        auto replacement_groups = collision_groups_;
        CollisionGroupId replacement_next_group_id = next_collision_group_id_;

        if (bulk_rebuild)
        {
            std::vector<CollisionPrimitiveGroup> groups;
            replacement_groups.clear();
            groups.reserve(working.size());
            for (const auto &[key, face] : working)
            {
                CollisionGroupId group{};
                if (CollisionBoundaryPolicy{}.isObstacle(
                        face.boundary_kind,
                        CollisionSurfaceOrigin::GeneratedBoundary))
                {
                    const auto triangles = collisionTrianglesForFace(
                        face, face.source_face_id);
                    if (!triangles.hasValue())
                        return Result<std::monostate, SpatialError>::failure(
                            triangles.error());
                    group = replacement_next_group_id++;
                    groups.push_back({group, triangles.value()});
                }
                replacement_groups.emplace(key, group);
            }
            auto built = IncrementalCollisionIndex::build(std::move(groups));
            if (!built.hasValue())
                return Result<std::monostate, SpatialError>::failure(
                    built.error());
            replacement_index = std::move(built.value());
        }
        else
        {
            for (const BoundaryFaceKey &key : update.erase_faces)
            {
                const auto found = replacement_groups.find(key);
                if (found->second != CollisionGroupId{})
                {
                    const auto erased = replacement_index.eraseGroup(found->second);
                    if (!erased.hasValue())
                        return Result<std::monostate, SpatialError>::failure(
                            erased.error());
                }
                replacement_groups.erase(found);
            }
            for (PendingInsert &entry : pending)
            {
                CollisionGroupId group{};
                if (entry.indexed)
                {
                    group = replacement_next_group_id++;
                    const auto inserted = replacement_index.insertGroup(
                        {group, std::move(entry.triangles)});
                    if (!inserted.hasValue())
                        return Result<std::monostate, SpatialError>::failure(
                            inserted.error());
                }
                replacement_groups.emplace(entry.key, group);
            }
            replacement_index.compactInactive();
            const auto rebuilt = replacement_index.rebuildIfDegraded();
            if (!rebuilt.hasValue())
                return Result<std::monostate, SpatialError>::failure(
                    rebuilt.error());
        }

        const auto index_end = std::chrono::steady_clock::now();
        collision_index_ = std::move(replacement_index);
        collision_groups_ = std::move(replacement_groups);
        next_collision_group_id_ = replacement_next_group_id;
        faces_ = mapFaces(working);
        last_apply_diagnostics_ = {
            update.erase_faces.size(),
            update.insert_faces.size(),
            bulk_rebuild,
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    preparation_end - preparation_start).count()),
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    index_end - index_start).count())};
        return Result<std::monostate, SpatialError>::success({});
    }

    const IncrementalCollisionIndex &
    ExposedBoundaryTracker::collisionIndex() const noexcept
    {
        return collision_index_;
    }

    const ExposedBoundaryApplyDiagnostics &
    ExposedBoundaryTracker::lastApplyDiagnostics() const noexcept
    {
        return last_apply_diagnostics_;
    }

    std::size_t ExposedBoundaryTracker::faceCount() const noexcept
    {
        return faces_.size();
    }

    bool ExposedBoundaryTracker::contains(
        const BoundaryFaceKey &key) const noexcept
    {
        return collision_groups_.find(key) != collision_groups_.end();
    }

    const std::vector<BoundaryFace> &
    ExposedBoundaryTracker::faces() const noexcept
    {
        return faces_;
    }

    Result<std::vector<CollisionTriangle>, SpatialError>
    ExposedBoundaryTracker::collisionTriangles(
        const CollisionBoundaryPolicy &policy) const
    {
        std::vector<CollisionTriangle> triangles;
        for (std::size_t face_index = 0; face_index < faces_.size(); ++face_index)
        {
            const BoundaryFace &face = faces_[face_index];
            if (!policy.isObstacle(
                    face.boundary_kind,
                    CollisionSurfaceOrigin::GeneratedBoundary))
                continue;
            const std::array<std::array<std::size_t, 3>, 2> splits{{
                {{0, 1, 2}}, {{0, 2, 3}}}};
            const std::size_t split_count = face.points.size() == 3 ? 1 : 2;
            for (std::size_t split = 0; split < split_count; ++split)
            {
                CollisionTriangle triangle;
                triangle.owner_kind = CollisionOwnerKind::ExposedBoundary;
                triangle.owner_id = static_cast<std::uint32_t>(face_index);
                triangle.boundary_vertex_count =
                    static_cast<std::uint8_t>(face.points.size());
                for (std::size_t boundary = 0;
                     boundary < face.points.size();
                     ++boundary)
                {
                    triangle.boundary_points[boundary] = face.points[boundary];
                    triangle.boundary_vertex_keys[boundary] =
                        face.vertex_keys[boundary];
                }
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const std::size_t index = splits[split][corner];
                    triangle.points[corner] = face.points[index];
                    triangle.vertex_keys[corner] = face.vertex_keys[index];
                }
                if ((triangle.points[1] - triangle.points[0])
                        .cross(triangle.points[2] - triangle.points[0])
                        .squaredNorm() == Scalar{0})
                {
                    return Result<std::vector<CollisionTriangle>, SpatialError>::failure(
                        SpatialError::DegenerateTriangle);
                }
                triangles.push_back(triangle);
            }
        }
        return Result<std::vector<CollisionTriangle>, SpatialError>::success(
            std::move(triangles));
    }
}
