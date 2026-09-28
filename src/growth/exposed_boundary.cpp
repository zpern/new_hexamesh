#include <boundary_mesh/growth/exposed_boundary.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <optional>
#include <unordered_map>
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

        struct EdgeKey
        {
            CollisionVertexKey first;
            CollisionVertexKey second;
        };

        EdgeKey edgeKey(
            CollisionVertexKey first,
            CollisionVertexKey second)
        {
            if (keyLess(second, first)) std::swap(first, second);
            return {first, second};
        }

        struct EdgeKeyHash
        {
            std::size_t operator()(const EdgeKey &key) const noexcept
            {
                std::size_t seed = 0;
                const auto combine = [&](std::size_t value)
                {
                    seed ^= value + static_cast<std::size_t>(0x9e3779b9) +
                        (seed << 6) + (seed >> 2);
                };
                for (const CollisionVertexKey *vertex :
                     {&key.first, &key.second})
                {
                    combine(std::hash<VertexId>{}(vertex->source_vertex_id));
                    combine(std::hash<std::uint32_t>{}(vertex->layer));
                    combine(std::hash<std::uint32_t>{}(vertex->branch_id));
                }
                return seed;
            }
        };

        struct EdgeKeyEqual
        {
            bool operator()(const EdgeKey &left, const EdgeKey &right) const
            {
                return keyEqual(left.first, right.first) &&
                    keyEqual(left.second, right.second);
            }
        };

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

        BoundaryFace candidateSideFace(
            const LayerBoundaryCandidate &candidate,
            std::size_t index)
        {
            const std::size_t count = candidate.bottom.points.size();
            const std::size_t next = (index + 1) % count;
            return BoundaryFace{
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
                    : candidate.top.boundary_kind};
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

    Result<std::monostate, SpatialError>
    ExposedBoundaryTracker::initializeWallSurface(
        const SurfaceMesh &surface)
    {
        if (!faces_by_key_.empty() ||
            surface.faces.size() != surface.face_tags.size() ||
            surface.faces.size() > static_cast<std::size_t>(
                std::numeric_limits<SurfaceFaceId>::max()))
            return Result<std::monostate, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);

        ExposedBoundaryUpdate seed;
        std::set<BoundaryFaceKey, BoundaryFaceKeyLess> input_keys;
        for (std::size_t face_index = 0;
             face_index < surface.faces.size();
             ++face_index)
        {
            if (surface.face_tags[face_index].kind !=
                SurfaceBoundaryKind::Wall)
                continue;
            BoundaryFace face;
            face.source_face_id = static_cast<SurfaceFaceId>(face_index);
            face.region_id = surface.face_tags[face_index].region_id;
            face.boundary_kind = SurfaceBoundaryKind::Wall;
            const bool valid = std::visit(
                [&](const auto &source_face)
                {
                    for (const VertexId vertex_id : source_face.vertex_ids)
                    {
                        const std::size_t vertex =
                            static_cast<std::size_t>(vertex_id);
                        if (vertex >= surface.vertices.size()) return false;
                        face.points.push_back(surface.vertices[vertex]);
                        face.vertex_keys.push_back(
                            CollisionVertexKey{vertex_id, 0, 0});
                    }
                    return true;
                },
                surface.faces[face_index]);
            if (!valid)
                return Result<std::monostate, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            const auto key = makeBoundaryFaceKey(face);
            if (!key.hasValue())
                return Result<std::monostate, SpatialError>::failure(
                    key.error());
            if (!input_keys.insert(key.value()).second)
                return Result<std::monostate, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            seed.insert_faces.push_back(std::move(face));
        }
        const auto seeded = apply(seed);
        if (!seeded.hasValue()) return seeded;
        input_face_keys_ = std::move(input_keys);
        faces_cache_valid_ = false;
        return Result<std::monostate, SpatialError>::success({});
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
        struct TouchedFace
        {
            bool present{};
            BoundaryFace face;
        };
        struct SideIncidence
        {
            std::size_t count{};
            BoundaryFaceKey first_key;
        };
        std::unordered_map<EdgeKey, SideIncidence, EdgeKeyHash, EdgeKeyEqual>
            side_incidence;
        side_incidence.reserve(candidates.size() * 4);
        for (const LayerBoundaryCandidate &candidate : candidates)
        {
            if (!validFace(candidate.bottom) ||
                !validFace(candidate.top) ||
                candidate.bottom.points.size() != candidate.top.points.size())
                return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            const std::size_t count = candidate.bottom.vertex_keys.size();
            for (std::size_t side = 0; side < count; ++side)
            {
                const std::size_t next = (side + 1) % count;
                const auto side_key = makeBoundaryFaceKey(
                    candidateSideFace(candidate, side));
                if (!side_key.hasValue())
                    return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                        side_key.error());
                auto &incidence = side_incidence[edgeKey(
                    candidate.bottom.vertex_keys[side],
                    candidate.bottom.vertex_keys[next])];
                if (incidence.count == 0)
                    incidence.first_key = side_key.value();
                if (++incidence.count > 2)
                    return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
            }
        }
        std::map<BoundaryFaceKey, TouchedFace, BoundaryFaceKeyLess> touched;
        const auto stateFor = [&](const BoundaryFaceKey &key)
            -> TouchedFace &
        {
            auto found = touched.find(key);
            if (found != touched.end()) return found->second;
            const bool present = collision_groups_.find(key) !=
                collision_groups_.end();
            return touched.emplace(key, TouchedFace{present, {}})
                .first->second;
        };
        const auto toggle = [&](const BoundaryFace &face)
            -> Result<std::monostate, SpatialError>
        {
            const auto key = makeBoundaryFaceKey(face);
            if (!key.hasValue())
                return Result<std::monostate, SpatialError>::failure(
                    key.error());
            TouchedFace &state = stateFor(key.value());
            if (state.present)
            {
                state.present = false;
                state.face = {};
            }
            else
            {
                state.present = true;
                state.face = face;
            }
            return Result<std::monostate, SpatialError>::success({});
        };
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
            stateFor(bottom_key.value()).present = false;
            const auto toggled_top = toggle(candidate.top);
            if (!toggled_top.hasValue())
                return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                    toggled_top.error());
            const std::size_t count = candidate.bottom.vertex_keys.size();
            for (std::size_t side = 0; side < count; ++side)
            {
                const std::size_t next = (side + 1) % count;
                const BoundaryFace face = candidateSideFace(candidate, side);
                const auto key = makeBoundaryFaceKey(face);
                if (!key.hasValue())
                    return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                        key.error());
                const auto &incidence = side_incidence.at(edgeKey(
                    candidate.bottom.vertex_keys[side],
                    candidate.bottom.vertex_keys[next]));
                if (incidence.count == 2 &&
                    faceKeyEqual(incidence.first_key, key.value()))
                    continue;
                const auto toggled = toggle(face);
                if (!toggled.hasValue())
                    return Result<ExposedBoundaryUpdate, SpatialError>::failure(
                        toggled.error());
            }
        }
        ExposedBoundaryUpdate update;
        for (const auto &[key, state] : touched)
        {
            const bool originally_present = collision_groups_.find(key) !=
                collision_groups_.end();
            if (originally_present && !state.present)
                update.erase_faces.push_back(key);
            else if (!originally_present && state.present)
                update.insert_faces.push_back(state.face);
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
            CollisionGroupId group{};
        };
        std::vector<PendingInsert> pending;
        pending.reserve(update.insert_faces.size());
        std::set<BoundaryFaceKey, BoundaryFaceKeyLess> erase_keys;
        std::set<BoundaryFaceKey, BoundaryFaceKeyLess> insert_keys;

        for (const BoundaryFaceKey &key : update.erase_faces)
        {
            if (faces_by_key_.find(key) == faces_by_key_.end() ||
                collision_groups_.find(key) == collision_groups_.end() ||
                !erase_keys.insert(key).second)
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
            if ((faces_by_key_.find(key.value()) != faces_by_key_.end() &&
                 erase_keys.find(key.value()) == erase_keys.end()) ||
                !insert_keys.insert(key.value()).second)
                return Result<std::monostate, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            pending.push_back(
                {key.value(), face, std::move(triangles), indexed, {}});
        }

        const std::size_t changed_faces = std::max(
            update.erase_faces.size(), update.insert_faces.size());
        const bool bulk_rebuild = faces_by_key_.empty() ||
            changed_faces * 5 >= faces_by_key_.size();
        const auto preparation_end = std::chrono::steady_clock::now();
        const auto index_start = preparation_end;

        CollisionGroupId replacement_next_group_id = next_collision_group_id_;
        std::optional<IncrementalCollisionIndex> replacement_index;

        if (bulk_rebuild)
        {
            std::vector<CollisionPrimitiveGroup> groups;
            groups.reserve(
                faces_by_key_.size() - erase_keys.size() + pending.size() +
                transition_groups_.size());
            for (const auto &[group, triangles] : transition_groups_)
                groups.push_back({group, triangles});
            for (const auto &[key, face] : faces_by_key_)
            {
                if (erase_keys.find(key) != erase_keys.end()) continue;
                const CollisionGroupId group = collision_groups_.at(key);
                if (group != CollisionGroupId{})
                {
                    const auto triangles = collisionTrianglesForFace(
                        face, face.source_face_id);
                    if (!triangles.hasValue())
                        return Result<std::monostate, SpatialError>::failure(
                            triangles.error());
                    groups.push_back({group, triangles.value()});
                }
            }
            for (PendingInsert &entry : pending)
                if (entry.indexed)
                {
                    entry.group = replacement_next_group_id++;
                    groups.push_back({entry.group, entry.triangles});
                }
            auto built = IncrementalCollisionIndex::build(std::move(groups));
            if (!built.hasValue())
                return Result<std::monostate, SpatialError>::failure(
                    built.error());
            replacement_index.emplace(std::move(built.value()));
        }
        else
        {
            for (const BoundaryFaceKey &key : update.erase_faces)
            {
                const CollisionGroupId group = collision_groups_.at(key);
                if (group != CollisionGroupId{})
                {
                    const auto erased = collision_index_.eraseGroup(group);
                    if (!erased.hasValue())
                        return Result<std::monostate, SpatialError>::failure(
                            erased.error());
                }
            }
            for (PendingInsert &entry : pending)
            {
                if (entry.indexed)
                {
                    entry.group = replacement_next_group_id++;
                    const auto inserted = collision_index_.insertGroup(
                        {entry.group, std::move(entry.triangles)});
                    if (!inserted.hasValue())
                        return Result<std::monostate, SpatialError>::failure(
                            inserted.error());
                }
            }
            const auto rebuilt = collision_index_.rebuildIfDegraded();
            if (!rebuilt.hasValue())
                return Result<std::monostate, SpatialError>::failure(
                    rebuilt.error());
        }

        const auto index_end = std::chrono::steady_clock::now();
        if (replacement_index.has_value())
            collision_index_ = std::move(*replacement_index);
        for (const BoundaryFaceKey &key : update.erase_faces)
        {
            collision_groups_.erase(key);
            faces_by_key_.erase(key);
            input_face_keys_.erase(key);
        }
        for (const PendingInsert &entry : pending)
        {
            faces_by_key_.emplace(entry.key, entry.face);
            collision_groups_.emplace(entry.key, entry.group);
        }
        next_collision_group_id_ = replacement_next_group_id;
        faces_cache_valid_ = false;
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

    Result<std::monostate, SpatialError>
    ExposedBoundaryTracker::appendTransitionTriangles(
        std::vector<CollisionTriangle> triangles)
    {
        if (triangles.empty())
            return Result<std::monostate, SpatialError>::success({});
        if (next_collision_group_id_ == CollisionGroupId{})
            return Result<std::monostate, SpatialError>::failure(
                SpatialError::PrimitiveIdOverflow);
        const CollisionGroupId group = next_collision_group_id_++;
        const auto inserted = collision_index_.insertGroup({
            group, triangles});
        if (!inserted.hasValue())
        {
            --next_collision_group_id_;
            return inserted;
        }
        transition_groups_.emplace(group, std::move(triangles));
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
        return faces_by_key_.size();
    }

    bool ExposedBoundaryTracker::contains(
        const BoundaryFaceKey &key) const noexcept
    {
        return faces_by_key_.find(key) != faces_by_key_.end();
    }

    const std::vector<BoundaryFace> &
    ExposedBoundaryTracker::faces() const
    {
        if (!faces_cache_valid_)
        {
            faces_cache_.clear();
            faces_cache_.reserve(
                faces_by_key_.size() - input_face_keys_.size());
            for (const auto &[key, face] : faces_by_key_)
            {
                if (input_face_keys_.find(key) != input_face_keys_.end())
                    continue;
                faces_cache_.push_back(face);
            }
            faces_cache_valid_ = true;
        }
        return faces_cache_;
    }

    Result<std::vector<CollisionTriangle>, SpatialError>
    ExposedBoundaryTracker::collisionTriangles(
        const CollisionBoundaryPolicy &policy) const
    {
        std::vector<CollisionTriangle> triangles;
        std::size_t face_index = 0;
        for (const auto &[key, face] : faces_by_key_)
        {
            if (input_face_keys_.find(key) != input_face_keys_.end())
                continue;
            const std::size_t current_face_index = face_index++;
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
                triangle.owner_id =
                    static_cast<std::uint32_t>(current_face_index);
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
