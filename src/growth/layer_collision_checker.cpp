#include <boundary_mesh/growth/layer_collision_checker.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        std::vector<VertexId> faceVertexIds(const SurfaceFace &face)
        {
            return std::visit(
                [](const auto &value)
                {
                    return std::vector<VertexId>{
                        value.vertex_ids.begin(),
                        value.vertex_ids.end()};
                },
                face);
        }

        Result<std::vector<CollisionTriangle>, SpatialError>
        candidateTriangles(
            const LayerBoundaryCandidate &candidate,
            std::uint32_t owner_id)
        {
            std::vector<BoundaryFace> faces{candidate.top};
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

            std::vector<CollisionTriangle> triangles;
            for (const BoundaryFace &face : faces)
            {
                if (face.points.size() != face.vertex_keys.size() ||
                    (face.points.size() != 3 && face.points.size() != 4))
                {
                    return Result<std::vector<CollisionTriangle>, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
                }
                const std::array<std::array<std::size_t, 3>, 2> splits{{
                    {{0, 1, 2}}, {{0, 2, 3}}}};
                const std::size_t split_count =
                    face.points.size() == 3 ? 1 : 2;
                for (std::size_t split = 0; split < split_count; ++split)
                {
                    CollisionTriangle triangle;
                    triangle.owner_kind = CollisionOwnerKind::LayerCandidate;
                    triangle.owner_id = owner_id;
                    triangle.boundary_vertex_count =
                        static_cast<std::uint8_t>(face.points.size());
                    for (std::size_t boundary = 0;
                         boundary < face.points.size();
                         ++boundary)
                    {
                        triangle.boundary_points[boundary] =
                            face.points[boundary];
                        triangle.boundary_vertex_keys[boundary] =
                            face.vertex_keys[boundary];
                    }
                    for (std::size_t corner = 0; corner < 3; ++corner)
                    {
                        const std::size_t index = splits[split][corner];
                        triangle.points[corner] = face.points[index];
                        triangle.vertex_keys[corner] = face.vertex_keys[index];
                    }
                    const auto bounds = makeAabb(
                        triangle.points[0],
                        triangle.points[1],
                        triangle.points[2]);
                    if (!bounds.hasValue())
                    {
                        return Result<std::vector<CollisionTriangle>, SpatialError>::failure(
                            bounds.error());
                    }
                    triangles.push_back(triangle);
                }
            }
            return Result<std::vector<CollisionTriangle>, SpatialError>::success(
                std::move(triangles));
        }

        LayerStepResult compactStep(
            const GrowthFront &current_front,
            const LayerStepResult &input,
            const std::vector<bool> &stopped)
        {
            LayerStepResult output;
            output.layer = input.layer;
            output.next_front.layer = input.next_front.layer;
            output.stopped_faces = input.stopped_faces;
            output.completed_faces = input.completed_faces;

            std::vector<bool> used(input.next_front.vertices.size(), false);
            for (std::size_t face_index = 0;
                 face_index < input.next_front.faces.size();
                 ++face_index)
            {
                if (stopped[face_index])
                {
                    const std::size_t previous =
                        input.previous_front_face_indices[face_index];
                    output.stopped_faces.push_back(
                        {previous,
                         current_front.source_face_ids[previous],
                         input.layer,
                         FaceStopReason::Collision});
                    continue;
                }
                for (const VertexId id :
                     faceVertexIds(input.next_front.faces[face_index]))
                {
                    used[static_cast<std::size_t>(id)] = true;
                }
            }

            std::vector<VertexId> remap(
                input.next_front.vertices.size(), VertexId{});
            for (std::size_t vertex = 0;
                 vertex < input.next_front.vertices.size();
                 ++vertex)
            {
                if (!used[vertex])
                {
                    continue;
                }
                remap[vertex] = static_cast<VertexId>(
                    output.next_front.vertices.size());
                output.next_front.vertices.push_back(
                    input.next_front.vertices[vertex]);
                output.next_front.source_vertex_ids.push_back(
                    input.next_front.source_vertex_ids[vertex]);
                output.next_front.vertex_boundaries.push_back(
                    input.next_front.vertex_boundaries[vertex]);
                output.previous_front_vertex_indices.push_back(
                    input.previous_front_vertex_indices[vertex]);
            }

            for (std::size_t face_index = 0;
                 face_index < input.next_front.faces.size();
                 ++face_index)
            {
                if (stopped[face_index])
                {
                    continue;
                }
                SurfaceFace face = std::visit(
                    [&](const auto &value) -> SurfaceFace
                    {
                        auto remapped = value;
                        for (VertexId &id : remapped.vertex_ids)
                        {
                            id = remap[static_cast<std::size_t>(id)];
                        }
                        return remapped;
                    },
                    input.next_front.faces[face_index]);
                output.next_front.faces.push_back(std::move(face));
                output.next_front.source_face_ids.push_back(
                    input.next_front.source_face_ids[face_index]);
                output.previous_front_face_indices.push_back(
                    input.previous_front_face_indices[face_index]);
            }
            std::sort(
                output.stopped_faces.begin(),
                output.stopped_faces.end(),
                [](const FaceStopEvent &left, const FaceStopEvent &right)
                {
                    return left.source_face_id < right.source_face_id;
                });
            return output;
        }

        bool hitsIndex(
            const std::vector<CollisionTriangle> &triangles,
            const CollisionIndex &index)
        {
            return std::any_of(
                triangles.begin(),
                triangles.end(),
                [&](const CollisionTriangle &triangle)
                {
                    return !index.queryIllegalContacts(triangle).empty();
                });
        }

        bool sameKey(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right)
        {
            return left.source_vertex_id == right.source_vertex_id &&
                   left.layer == right.layer;
        }

        bool containsKey(
            const std::vector<CollisionVertexKey> &keys,
            const CollisionVertexKey &key)
        {
            return std::any_of(
                keys.begin(),
                keys.end(),
                [&](const CollisionVertexKey &value)
                {
                    return sameKey(value, key);
                });
        }

        bool adjacentKeys(
            const std::vector<CollisionVertexKey> &keys,
            const CollisionVertexKey &first,
            const CollisionVertexKey &second)
        {
            for (std::size_t index = 0; index < keys.size(); ++index)
            {
                if (!sameKey(keys[index], first)) continue;
                const std::size_t previous =
                    (index + keys.size() - 1) % keys.size();
                const std::size_t next = (index + 1) % keys.size();
                return sameKey(keys[previous], second) ||
                       sameKey(keys[next], second);
            }
            return false;
        }

        std::optional<std::array<CollisionVertexKey, 4>> sharedSideKeys(
            const LayerBoundaryCandidate &first,
            const LayerBoundaryCandidate &second)
        {
            std::vector<CollisionVertexKey> shared_bottom;
            for (const CollisionVertexKey &key : first.bottom.vertex_keys)
            {
                if (containsKey(second.bottom.vertex_keys, key))
                {
                    shared_bottom.push_back(key);
                }
            }
            if (shared_bottom.size() != 2 ||
                !adjacentKeys(
                    first.bottom.vertex_keys,
                    shared_bottom[0],
                    shared_bottom[1]) ||
                !adjacentKeys(
                    second.bottom.vertex_keys,
                    shared_bottom[0],
                    shared_bottom[1]))
            {
                return std::nullopt;
            }

            std::array<CollisionVertexKey, 4> side{
                shared_bottom[0],
                shared_bottom[1],
                CollisionVertexKey{},
                CollisionVertexKey{}};
            for (std::size_t endpoint = 0; endpoint < 2; ++endpoint)
            {
                const auto found = std::find_if(
                    first.top.vertex_keys.begin(),
                    first.top.vertex_keys.end(),
                    [&](const CollisionVertexKey &key)
                    {
                        return key.source_vertex_id ==
                            shared_bottom[endpoint].source_vertex_id;
                    });
                if (found == first.top.vertex_keys.end() ||
                    !containsKey(second.top.vertex_keys, *found))
                {
                    return std::nullopt;
                }
                side[endpoint + 2] = *found;
            }
            return side;
        }

        bool triangleUsesOnly(
            const CollisionTriangle &triangle,
            const std::array<CollisionVertexKey, 4> &side)
        {
            return std::all_of(
                triangle.vertex_keys.begin(),
                triangle.vertex_keys.end(),
                [&](const CollisionVertexKey &key)
                {
                    return std::any_of(
                        side.begin(),
                        side.end(),
                        [&](const CollisionVertexKey &side_key)
                        {
                            return sameKey(key, side_key);
                        });
                });
        }

        bool legalSharedSideContact(
            const std::vector<LayerBoundaryCandidate> &candidates,
            const CollisionTriangle &first,
            const CollisionTriangle &second)
        {
            if (first.owner_id >= candidates.size() ||
                second.owner_id >= candidates.size())
            {
                return false;
            }
            const auto side = sharedSideKeys(
                candidates[first.owner_id],
                candidates[second.owner_id]);
            return side.has_value() &&
                   triangleUsesOnly(first, *side) &&
                   triangleUsesOnly(second, *side);
        }
    }

    Result<std::vector<LayerBoundaryCandidate>, SpatialError>
    buildLayerBoundaryCandidates(
        const GrowthFront &current_front,
        const LayerStepResult &step)
    {
        if (step.next_front.faces.size() !=
                step.previous_front_face_indices.size() ||
            step.next_front.vertices.size() !=
                step.previous_front_vertex_indices.size())
        {
            return Result<std::vector<LayerBoundaryCandidate>, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        }

        std::vector<LayerBoundaryCandidate> candidates;
        candidates.reserve(step.next_front.faces.size());
        for (std::size_t face_index = 0;
             face_index < step.next_front.faces.size();
             ++face_index)
        {
            const std::size_t previous_face =
                step.previous_front_face_indices[face_index];
            if (previous_face >= current_front.faces.size())
            {
                return Result<std::vector<LayerBoundaryCandidate>, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            }
            const std::vector<VertexId> top_ids =
                faceVertexIds(step.next_front.faces[face_index]);
            BoundaryFace bottom;
            BoundaryFace top;
            bottom.source_face_id =
                current_front.source_face_ids[previous_face];
            top.source_face_id = bottom.source_face_id;
            for (const VertexId top_id : top_ids)
            {
                const std::size_t top_index =
                    static_cast<std::size_t>(top_id);
                if (top_index >= step.next_front.vertices.size())
                {
                    return Result<std::vector<LayerBoundaryCandidate>, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
                }
                const std::size_t bottom_index =
                    step.previous_front_vertex_indices[top_index];
                if (bottom_index >= current_front.vertices.size())
                {
                    return Result<std::vector<LayerBoundaryCandidate>, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
                }
                bottom.points.push_back(current_front.vertices[bottom_index]);
                bottom.vertex_keys.push_back(
                    {current_front.source_vertex_ids[bottom_index],
                     current_front.layer});
                top.points.push_back(step.next_front.vertices[top_index]);
                top.vertex_keys.push_back(
                    {step.next_front.source_vertex_ids[top_index],
                     step.next_front.layer});
            }
            candidates.push_back({std::move(bottom), std::move(top)});
        }
        return Result<std::vector<LayerBoundaryCandidate>, SpatialError>::success(
            std::move(candidates));
    }

    Result<LayerStepResult, SpatialError>
    LayerCollisionChecker::filterAgainstObstacles(
        const CollisionIndex &original_surface,
        const ExposedBoundaryTracker &exposed_boundary,
        const GrowthFront &current_front,
        const LayerStepResult &quality_step) const
    {
        const auto candidates = buildLayerBoundaryCandidates(
            current_front, quality_step);
        if (!candidates.hasValue())
        {
            return Result<LayerStepResult, SpatialError>::failure(
                candidates.error());
        }
        const auto history_triangles = exposed_boundary.collisionTriangles();
        if (!history_triangles.hasValue())
        {
            return Result<LayerStepResult, SpatialError>::failure(
                history_triangles.error());
        }
        const auto history = CollisionIndex::build(history_triangles.value());
        if (!history.hasValue())
        {
            return Result<LayerStepResult, SpatialError>::failure(
                history.error());
        }

        std::vector<bool> stopped(candidates.value().size(), false);
        for (std::size_t index = 0; index < candidates.value().size(); ++index)
        {
            const auto triangles = candidateTriangles(
                candidates.value()[index],
                static_cast<std::uint32_t>(index));
            if (!triangles.hasValue())
            {
                return Result<LayerStepResult, SpatialError>::failure(
                    triangles.error());
            }
            stopped[index] = hitsIndex(triangles.value(), original_surface) ||
                             hitsIndex(triangles.value(), history.value());
        }
        return Result<LayerStepResult, SpatialError>::success(
            compactStep(current_front, quality_step, stopped));
    }

    Result<LayerStepResult, SpatialError>
    LayerCollisionChecker::filterSelfCollisions(
        const GrowthFront &current_front,
        const LayerStepResult &obstacle_step) const
    {
        const auto candidates = buildLayerBoundaryCandidates(
            current_front, obstacle_step);
        if (!candidates.hasValue())
        {
            return Result<LayerStepResult, SpatialError>::failure(
                candidates.error());
        }

        std::vector<CollisionTriangle> all;
        for (std::size_t index = 0; index < candidates.value().size(); ++index)
        {
            const auto triangles = candidateTriangles(
                candidates.value()[index],
                static_cast<std::uint32_t>(index));
            if (!triangles.hasValue())
            {
                return Result<LayerStepResult, SpatialError>::failure(
                    triangles.error());
            }
            all.insert(
                all.end(),
                triangles.value().begin(),
                triangles.value().end());
        }
        const auto index = CollisionIndex::build(all);
        if (!index.hasValue())
        {
            return Result<LayerStepResult, SpatialError>::failure(
                index.error());
        }

        std::vector<bool> stopped(candidates.value().size(), false);
        for (std::size_t primitive = 0; primitive < all.size(); ++primitive)
        {
            const std::uint32_t owner = all[primitive].owner_id;
            for (const std::size_t hit :
                 index.value().queryIllegalContacts(all[primitive]))
            {
                const std::uint32_t other =
                    index.value().primitive(hit).owner_id;
                if (owner != other &&
                    !legalSharedSideContact(
                        candidates.value(),
                        all[primitive],
                        index.value().primitive(hit)))
                {
                    stopped[owner] = true;
                    stopped[other] = true;
                }
            }
        }
        return Result<LayerStepResult, SpatialError>::success(
            compactStep(current_front, obstacle_step, stopped));
    }
}
