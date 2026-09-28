#include <boundary_mesh/growth/layer_collision_checker.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <iostream>
#include <unordered_map>
#include <utility>

#include <boundary_mesh/spatial/batch_self_collision_detector.hpp>

namespace boundary_mesh
{
    namespace
    {
        std::vector<Aabb> conservativeOwnerBounds(
            const LayerBoundaryBatch &batch)
        {
            std::vector<Aabb> bounds;
            bounds.reserve(batch.candidates().size());
            for (const LayerBoundaryCandidate &candidate : batch.candidates())
            {
                // Every possible side face lies inside the full bottom/top
                // prism envelope, even when this side is currently shared.
                const Point3 &first = candidate.bottom.points.front();
                Aabb envelope{first, first};
                for (const Point3 &point : candidate.bottom.points)
                {
                    envelope.minimum = envelope.minimum.cwiseMin(point);
                    envelope.maximum = envelope.maximum.cwiseMax(point);
                }
                for (const Point3 &point : candidate.top.points)
                {
                    envelope.minimum = envelope.minimum.cwiseMin(point);
                    envelope.maximum = envelope.maximum.cwiseMax(point);
                }
                bounds.push_back(std::move(envelope));
            }
            return bounds;
        }

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
            for (const FaceStopEvent &event :
                 input.accepted_stopped_faces)
            {
                if (std::find(
                        output.next_front.source_face_ids.begin(),
                        output.next_front.source_face_ids.end(),
                        event.source_face_id) !=
                    output.next_front.source_face_ids.end())
                {
                    output.accepted_stopped_faces.push_back(event);
                }
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

        bool hitsIndex(
            const std::vector<CollisionTriangle> &triangles,
            const IncrementalCollisionIndex &index)
        {
            return std::any_of(
                triangles.begin(), triangles.end(),
                [&](const CollisionTriangle &triangle)
                {
                    return !index.queryIllegalContacts(triangle).empty();
                });
        }

        Result<bool, SpatialError> hitsSlidingIndex(
            const LayerBoundaryCandidate &candidate,
            const SlidingIntersectionIndex &index)
        {
            struct SlidingQuery
            {
                TrianglePoints points;
                std::array<std::vector<std::uint32_t>, 3> associations;
                std::uint8_t physical_edges{};
                std::vector<std::uint32_t> complete_exemptions;
            };
            const auto commonRegions = [](const std::array<
                std::vector<std::uint32_t>, 4> &input)
            {
                std::array<std::vector<std::uint32_t>, 4> normalized = input;
                for (auto &regions : normalized)
                {
                    std::sort(regions.begin(), regions.end());
                    regions.erase(std::unique(regions.begin(), regions.end()),
                                  regions.end());
                }
                std::vector<std::uint32_t> common = normalized[0];
                for (std::size_t i = 1; i < 4 && !common.empty(); ++i)
                {
                    std::vector<std::uint32_t> next;
                    std::set_intersection(
                        common.begin(), common.end(),
                        normalized[i].begin(), normalized[i].end(),
                        std::back_inserter(next));
                    common.swap(next);
                }
                return common;
            };
            const auto validFace = [](const BoundaryFace &face)
            {
                return face.points.size() == face.vertex_sliding_region_ids.size() &&
                    (face.points.size() == 3 || face.points.size() == 4);
            };
            if (!validFace(candidate.bottom) || !validFace(candidate.top) ||
                candidate.bottom.points.size() != candidate.top.points.size())
                return Result<bool, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);

            std::vector<SlidingQuery> queries;
            const std::size_t count = candidate.top.points.size();
            const std::array<std::array<std::size_t, 3>, 2> top_splits{{
                {{0, 1, 2}}, {{0, 2, 3}}}};
            const std::size_t top_split_count = count == 3 ? 1 : 2;
            for (std::size_t split = 0; split < top_split_count; ++split)
            {
                SlidingQuery query;
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const std::size_t local = top_splits[split][corner];
                    query.points[corner] = candidate.top.points[local];
                    query.associations[corner] =
                        candidate.top.vertex_sliding_region_ids[local];
                }
                query.physical_edges = count == 3 ? 0b111 :
                    (split == 0 ? 0b011 : 0b110);
                queries.push_back(std::move(query));
            }
            for (std::size_t side = 0; side < count; ++side)
            {
                const std::size_t next = (side + 1) % count;
                const std::array<Point3, 4> points{{
                    candidate.bottom.points[side],
                    candidate.bottom.points[next],
                    candidate.top.points[next],
                    candidate.top.points[side]}};
                const std::array<std::vector<std::uint32_t>, 4> regions{{
                    candidate.bottom.vertex_sliding_region_ids[side],
                    candidate.bottom.vertex_sliding_region_ids[next],
                    candidate.top.vertex_sliding_region_ids[next],
                    candidate.top.vertex_sliding_region_ids[side]}};
                const std::vector<std::uint32_t> complete = commonRegions(regions);
                const std::array<std::array<std::size_t, 3>, 2> splits{{
                    {{0, 1, 2}}, {{0, 2, 3}}}};
                for (std::size_t split = 0; split < 2; ++split)
                {
                    SlidingQuery query;
                    for (std::size_t corner = 0; corner < 3; ++corner)
                    {
                        const std::size_t local = splits[split][corner];
                        query.points[corner] = points[local];
                        query.associations[corner] = regions[local];
                    }
                    query.physical_edges = split == 0 ? 0b011 : 0b110;
                    query.complete_exemptions = complete;
                    queries.push_back(std::move(query));
                }
            }

            for (const SlidingQuery &query : queries)
            {
                const auto permissions = buildSlidingContactPermissions(
                    query.associations, query.physical_edges,
                    query.complete_exemptions);
                auto hit = index.query(query.points, permissions);
                if (!hit.hasValue())
                    return Result<bool, SpatialError>::failure(hit.error());
                if (hit.value().intersected)
                {
                    const std::uint32_t region = hit.value().region_id;
                    std::vector<std::size_t> associated_columns;
                    for (std::size_t column = 0; column < count; ++column)
                    {
                        const auto contains = [region](const auto &regions) {
                            return std::find(regions.begin(), regions.end(), region) !=
                                regions.end();
                        };
                        if (contains(candidate.bottom.vertex_sliding_region_ids[column]) &&
                            contains(candidate.top.vertex_sliding_region_ids[column]))
                            associated_columns.push_back(column);
                    }
                    if (!associated_columns.empty() &&
                        associated_columns.size() < count)
                    {
                        const auto reference = index.faceNormalAtPoint(
                            region,
                            candidate.top.points[associated_columns.front()]);
                        bool valid = reference.hasValue();
                        std::vector<Scalar> sides;
                        for (std::size_t column = 0; valid && column < count; ++column)
                        {
                            if (std::find(associated_columns.begin(),
                                          associated_columns.end(), column) !=
                                associated_columns.end())
                                continue;
                            for (const Point3 *point : {
                                     &candidate.bottom.points[column],
                                     &candidate.top.points[column]})
                            {
                                const auto side_value = index.signedSideToRegion(
                                    region, *point, reference.value());
                                if (!side_value.hasValue())
                                {
                                    valid = false;
                                    break;
                                }
                                sides.push_back(side_value.value());
                            }
                        }
                        if (valid && slidingSideValuesStayOnOneSide(
                                sides, Scalar{1e-10}))
                        {
                            hit = index.query(
                                query.points, permissions, {region});
                            if (!hit.hasValue())
                                return Result<bool, SpatialError>::failure(
                                    hit.error());
                        }
                    }
                }
                if (hit.value().intersected)
                    return Result<bool, SpatialError>::success(true);
            }
            return Result<bool, SpatialError>::success(false);
        }

    }

    Result<std::vector<LayerBoundaryCandidate>, SpatialError>
    buildLayerBoundaryCandidates(
        const GrowthFront &current_front,
        const LayerStepResult &step)
    {
        return buildLayerBoundaryCandidates(
            current_front, step, SlidingSurfaceSet{});
    }

    Result<std::vector<LayerBoundaryCandidate>, SpatialError>
    buildLayerBoundaryCandidates(
        const GrowthFront &current_front,
        const LayerStepResult &step,
        const SlidingSurfaceSet &sliding_surfaces)
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
                bottom.points.push_back(
                    current_front.vertices[bottom_index].position);
                bottom.vertex_keys.push_back(
                    {current_front.vertices[bottom_index].source_vertex_id,
                     current_front.layer,
                     current_front.vertices[bottom_index].branch_id});
                bottom.vertex_sliding_region_ids.push_back(
                    current_front.vertices[bottom_index]
                        .boundary.sliding_region_ids);
                top.points.push_back(
                    step.next_front.vertices[top_index].position);
                top.vertex_keys.push_back(
                    {step.next_front.vertices[top_index].source_vertex_id,
                     step.next_front.layer,
                     step.next_front.vertices[top_index].branch_id});
                top.vertex_sliding_region_ids.push_back(
                    step.next_front.vertices[top_index]
                        .boundary.sliding_region_ids);
            }
            std::vector<SurfaceBoundaryTag> side_tags;
            side_tags.reserve(top_ids.size());
            for (std::size_t local = 0; local < top_ids.size(); ++local)
            {
                const std::size_t next = (local + 1) % top_ids.size();
                const std::size_t first_bottom =
                    step.previous_front_vertex_indices[
                        static_cast<std::size_t>(top_ids[local])];
                const std::size_t second_bottom =
                    step.previous_front_vertex_indices[
                        static_cast<std::size_t>(top_ids[next])];
                const auto &first = current_front.vertices[first_bottom]
                    .boundary.sliding_region_ids;
                const auto &second = current_front.vertices[second_bottom]
                    .boundary.sliding_region_ids;
                std::vector<std::uint32_t> shared;
                std::set_intersection(
                    first.begin(), first.end(), second.begin(), second.end(),
                    std::back_inserter(shared));
                SurfaceBoundaryTag tag{
                    SurfaceBoundaryKind::BoundaryLayerInterface, 0};
                if (!shared.empty())
                    if (const SlidingSurface *surface =
                            sliding_surfaces.find(shared.front()))
                        tag = {surface->boundary_kind, surface->region_id};
                side_tags.push_back(tag);
            }
            candidates.push_back({
                std::move(bottom), std::move(top), std::move(side_tags)});
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
        auto batch = LayerBoundaryBatch::build(candidates.value());
        if (!batch.hasValue())
            return Result<LayerStepResult, SpatialError>::failure(batch.error());
        return filterAgainstObstacles(
            original_surface, exposed_boundary, current_front, quality_step,
            batch.value());
    }

    Result<LayerStepResult, SpatialError>
    LayerCollisionChecker::filterAgainstObstacles(
        const CollisionIndex &original_surface,
        const ExposedBoundaryTracker &exposed_boundary,
        const GrowthFront &current_front,
        const LayerStepResult &quality_step,
        const LayerBoundaryBatch &batch) const
    {
        if (batch.owners().size() != quality_step.next_front.faces.size())
            return Result<LayerStepResult, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        std::vector<bool> stopped(batch.owners().size(), false);
        for (std::size_t index = 0; index < batch.owners().size(); ++index)
        {
            const auto &triangles = batch.owners()[index].triangles;
            stopped[index] = hitsIndex(triangles, original_surface) ||
                             hitsIndex(triangles,
                                       exposed_boundary.collisionIndex());
        }
        return Result<LayerStepResult, SpatialError>::success(
            compactStep(current_front, quality_step, stopped));
    }

    Result<LayerStepResult, SpatialError>
    LayerCollisionChecker::filterAgainstObstacles(
        const CollisionIndex &original_surface,
        const SlidingIntersectionIndex &sliding_surface,
        const SlidingSurfaceSet &sliding_surfaces,
        const ExposedBoundaryTracker &exposed_boundary,
        const GrowthFront &current_front,
        const LayerStepResult &quality_step) const
    {
        const auto candidates = buildLayerBoundaryCandidates(
            current_front, quality_step, sliding_surfaces);
        if (!candidates.hasValue())
            return Result<LayerStepResult, SpatialError>::failure(
                candidates.error());
        const auto batch = LayerBoundaryBatch::build(candidates.value());
        if (!batch.hasValue())
            return Result<LayerStepResult, SpatialError>::failure(batch.error());
        return filterAgainstObstacles(
            original_surface, sliding_surface, sliding_surfaces,
            exposed_boundary, current_front, quality_step, batch.value());
    }

    Result<LayerStepResult, SpatialError>
    LayerCollisionChecker::filterAgainstObstacles(
        const CollisionIndex &original_surface,
        const SlidingIntersectionIndex &sliding_surface,
        const SlidingSurfaceSet &sliding_surfaces,
        const ExposedBoundaryTracker &exposed_boundary,
        const GrowthFront &current_front,
        const LayerStepResult &quality_step,
        const LayerBoundaryBatch &batch) const
    {
        if (batch.owners().size() != quality_step.next_front.faces.size() ||
            batch.candidates().size() != batch.owners().size())
            return Result<LayerStepResult, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        std::vector<bool> stopped(batch.owners().size(), false);
        for (std::size_t index = 0; index < batch.owners().size(); ++index)
        {
            bool sliding_intersection = false;
            if (!sliding_surface.empty())
            {
                const auto sliding_hit = hitsSlidingIndex(
                    batch.candidates()[index], sliding_surface);
                if (!sliding_hit.hasValue())
                    return Result<LayerStepResult, SpatialError>::failure(
                        sliding_hit.error());
                sliding_intersection = sliding_hit.value();
            }
            const auto &triangles = batch.owners()[index].triangles;
            stopped[index] = hitsIndex(triangles, original_surface) ||
                hitsIndex(triangles, exposed_boundary.collisionIndex()) ||
                sliding_intersection;
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
        const auto batch = LayerBoundaryBatch::build(candidates.value());
        if (!batch.hasValue())
            return Result<LayerStepResult, SpatialError>::failure(batch.error());
        return filterSelfCollisions(
            current_front, obstacle_step, batch.value());
    }

    Result<LayerStepResult, SpatialError>
    LayerCollisionChecker::filterSelfCollisions(
        const GrowthFront &current_front,
        const LayerStepResult &obstacle_step,
        LayerBoundaryBatch &batch) const
    {
        if (batch.owners().size() != obstacle_step.next_front.faces.size())
            return Result<LayerStepResult, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        LayerStepResult remaining = obstacle_step;
        LayerBoundaryBatch &current_batch = batch;
        std::optional<BatchSelfCollisionIndex> persistent_index;
        std::vector<std::size_t> current_changed_owner_indices;
        std::unordered_map<SurfaceFaceId, std::size_t> current_owner_indices;
        std::vector<bool> stopped(batch.owners().size(), false);
        current_owner_indices.reserve(current_batch.candidates().size());
        for (std::size_t index = 0;
             index < current_batch.candidates().size();
             ++index)
        {
            const SurfaceFaceId source =
                current_batch.candidates()[index].top.source_face_id;
            current_owner_indices.emplace(source, index);
        }
        bool full_scan = true;
        std::size_t full_scan_rounds = 0;
        std::size_t local_scan_rounds = 0;
        std::size_t changed_owner_total = 0;
        std::uint64_t owner_pair_total = 0;
        std::uint64_t triangle_pair_total = 0;
        std::uint64_t triangle_aabb_rejections = 0;
        std::uint64_t exact_test_total = 0;
        std::uint64_t illegal_owner_pair_total = 0;
        while (!obstacle_step.next_front.faces.empty())
        {
            if (full_scan)
                ++full_scan_rounds;
            else
            {
                ++local_scan_rounds;
                changed_owner_total +=
                    current_changed_owner_indices.size();
            }
            if (!full_scan && !persistent_index.has_value())
            {
                const auto built = BatchSelfCollisionIndex::build(
                    batch.owners(), conservativeOwnerBounds(batch));
                if (!built.hasValue())
                    return Result<LayerStepResult, SpatialError>::failure(
                        built.error());
                persistent_index.emplace(built.value());
            }
            const auto collisions = full_scan
                ? BatchSelfCollisionDetector::detectOwners(
                      current_batch.owners())
                : persistent_index->detectChanged(
                      current_batch.owners(),
                      current_changed_owner_indices);
            if (!collisions.hasValue())
                return Result<LayerStepResult, SpatialError>::failure(
                    collisions.error());
            owner_pair_total += collisions.value().diagnostics.owner_pairs;
            triangle_pair_total += collisions.value().diagnostics.unique_pairs;
            triangle_aabb_rejections +=
                collisions.value().diagnostics.aabb_rejections;
            exact_test_total += collisions.value().diagnostics.exact_tests;
            illegal_owner_pair_total +=
                collisions.value().diagnostics.illegal_owner_pairs;
            if (collisions.value().illegal_owner_ids.empty())
                break;

            std::vector<std::size_t> newly_stopped;
            for (const std::uint32_t owner :
                 collisions.value().illegal_owner_ids)
            {
                const auto candidate = current_owner_indices.find(owner);
                if (candidate == current_owner_indices.end())
                    return Result<LayerStepResult, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
                if (!stopped[candidate->second])
                {
                    stopped[candidate->second] = true;
                    newly_stopped.push_back(candidate->second);
                }
            }
            if (newly_stopped.empty()) break;
            const auto updated = current_batch.deactivateOwners(newly_stopped);
            if (!updated.hasValue())
                return Result<LayerStepResult, SpatialError>::failure(
                    updated.error());
            current_changed_owner_indices = updated.value();
            if (current_changed_owner_indices.empty()) break;
            full_scan = false;
        }
        if (std::any_of(stopped.begin(), stopped.end(),
                        [](bool value) { return value; }))
            remaining = compactStep(current_front, obstacle_step, stopped);
        std::cerr << "temporary self-collision scans full_rounds="
                  << full_scan_rounds
                  << " local_rounds=" << local_scan_rounds
                  << " changed_owners=" << changed_owner_total
                  << " owner_pairs=" << owner_pair_total
                  << " triangle_pairs=" << triangle_pair_total
                  << " triangle_aabb_rejections="
                  << triangle_aabb_rejections
                  << " exact_tests=" << exact_test_total
                  << " illegal_owner_pairs=" << illegal_owner_pair_total
                  << '\n';
        return Result<LayerStepResult, SpatialError>::success(
            std::move(remaining));
    }

    Result<LayerStepResult, SpatialError>
    LayerCollisionChecker::filterSelfCollisions(
        const GrowthFront &current_front,
        const LayerStepResult &obstacle_step,
        const LayerBoundaryBatch &batch) const
    {
        LayerBoundaryBatch mutable_batch = batch;
        return filterSelfCollisions(
            current_front, obstacle_step, mutable_batch);
    }
}
