#include <boundary_mesh/growth/exposed_boundary.hpp>

#include <algorithm>
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
                    left.layer < right.layer);
        }

        bool keyEqual(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right)
        {
            return left.source_vertex_id == right.source_vertex_id &&
                   left.layer == right.layer;
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

        struct BoundaryFaceKeyLess
        {
            bool operator()(
                const BoundaryFaceKey &left,
                const BoundaryFaceKey &right) const
            {
                return faceKeyLess(left, right);
            }
        };

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
                    candidate.top.region_id});
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

    void ExposedBoundaryTracker::apply(
        const ExposedBoundaryUpdate &update)
    {
        BoundaryFaceMap working = makeFaceMap(faces_);
        for (const BoundaryFaceKey &key : update.erase_faces)
        {
            working.erase(key);
        }
        for (const BoundaryFace &face : update.insert_faces)
        {
            working[makeBoundaryFaceKey(face).value()] = face;
        }
        faces_ = mapFaces(working);
    }

    std::size_t ExposedBoundaryTracker::faceCount() const noexcept
    {
        return faces_.size();
    }

    bool ExposedBoundaryTracker::contains(
        const BoundaryFaceKey &key) const noexcept
    {
        return findFace(faces_, key) != faces_.size();
    }

    const std::vector<BoundaryFace> &
    ExposedBoundaryTracker::faces() const noexcept
    {
        return faces_;
    }

    Result<std::vector<CollisionTriangle>, SpatialError>
    ExposedBoundaryTracker::collisionTriangles() const
    {
        std::vector<CollisionTriangle> triangles;
        for (std::size_t face_index = 0; face_index < faces_.size(); ++face_index)
        {
            const BoundaryFace &face = faces_[face_index];
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
