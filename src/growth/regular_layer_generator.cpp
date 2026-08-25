#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/growth/farfield_boundary_builder.hpp>
#include <boundary_mesh/growth/layer_collision_checker.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/growth/regular_layer_stepper.hpp>
#include <boundary_mesh/growth/termination_propagator.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool validInitialMapping(
            const GrowthPatch &patch,
            const GrowthFront &front)
        {
            if (front.layer != 0 ||
                front.faces.size() != front.source_face_ids.size())
            {
                return false;
            }
            for (const GrowthFrontVertex &vertex : front.vertices)
            {
                const bool known = std::any_of(
                    patch.vertices().begin(), patch.vertices().end(),
                    [&](const PatchVertex &patch_vertex)
                    {
                        return patch_vertex.source_vertex_id ==
                            vertex.source_vertex_id;
                    });
                if (!known)
                {
                    return false;
                }
            }
            for (const SurfaceFaceId source_face_id : front.source_face_ids)
            {
                if (std::find(
                        patch.sourceFaceIds().begin(),
                        patch.sourceFaceIds().end(),
                        source_face_id) == patch.sourceFaceIds().end())
                {
                    return false;
                }
            }
            return true;
        }

        bool hasLayerAttempt(
            const GrowthFront &front,
            const FaceLayerConstraintTable &constraints)
        {
            return std::any_of(
                front.source_face_ids.begin(),
                front.source_face_ids.end(),
                [&](const SurfaceFaceId source_face_id)
                {
                    const FaceLayerConstraint *constraint =
                        constraints.find(source_face_id);
                    return constraint != nullptr &&
                           front.layer < constraint->allowed_layer_count;
                });
        }

        LayerVertexRecord *findLayerRecord(
            LayerVertexTable &records,
            VertexId source_vertex_id)
        {
            const auto found = std::find_if(
                records.begin(), records.end(),
                [&](const LayerVertexRecord &record)
                {
                    return record.source_vertex_id == source_vertex_id;
                });
            return found == records.end() ? nullptr : &*found;
        }

        VertexGrowthRecord *findVertexRecord(
            std::vector<VertexGrowthRecord> &records,
            VertexId source_vertex_id)
        {
            const auto found = std::find_if(
                records.begin(), records.end(),
                [&](const VertexGrowthRecord &record)
                {
                    return record.source_vertex_id == source_vertex_id;
                });
            return found == records.end() ? nullptr : &*found;
        }

        FaceGrowthRecord *findFaceRecord(
            std::vector<FaceGrowthRecord> &records,
            SurfaceFaceId source_face_id)
        {
            const auto found = std::find_if(
                records.begin(), records.end(),
                [&](const FaceGrowthRecord &record)
                {
                    return record.source_face_id == source_face_id;
                });
            return found == records.end() ? nullptr : &*found;
        }

        std::vector<FaceStopEvent> directQualityStops(
            const std::vector<FaceStopEvent> &events)
        {
            std::vector<FaceStopEvent> output;
            std::copy_if(
                events.begin(),
                events.end(),
                std::back_inserter(output),
                [](const FaceStopEvent &event)
                {
                    return event.reason != FaceStopReason::None &&
                           event.reason != FaceStopReason::VertexLayerLimit &&
                           event.reason !=
                               FaceStopReason::NeighborLayerConstraint &&
                           event.reason != FaceStopReason::Collision;
                });
            return output;
        }

        std::vector<FaceStopEvent> addedCollisionStops(
            const std::vector<FaceStopEvent> &after,
            const std::vector<FaceStopEvent> &before)
        {
            std::vector<FaceStopEvent> output;
            for (const FaceStopEvent &event : after)
            {
                if (event.reason != FaceStopReason::Collision)
                {
                    continue;
                }
                const bool existed = std::any_of(
                    before.begin(),
                    before.end(),
                    [&](const FaceStopEvent &previous)
                    {
                        return previous.source_face_id ==
                            event.source_face_id;
                    });
                if (!existed)
                {
                    output.push_back(event);
                }
            }
            return output;
        }

        bool appendCandidateCell(
            const SurfaceFace &previous_face,
            const SurfaceFace &next_face,
            const std::vector<VertexId> &previous_global_ids,
            const std::vector<VertexId> &next_global_ids,
            std::vector<VolumeCell> &cells)
        {
            return std::visit(
                [&](const auto &bottom) -> bool
                {
                    using Face = std::decay_t<decltype(bottom)>;
                    const Face *top = std::get_if<Face>(&next_face);
                    if (top == nullptr) return false;

                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        Prism cell;
                        for (std::size_t local = 0; local < 3; ++local)
                        {
                            cell.vertex_ids[local] = previous_global_ids[
                                static_cast<std::size_t>(
                                    bottom.vertex_ids[local])];
                            cell.vertex_ids[local + 3] = next_global_ids[
                                static_cast<std::size_t>(
                                    top->vertex_ids[local])];
                        }
                        cells.push_back(cell);
                    }
                    else
                    {
                        Hexa cell;
                        for (std::size_t local = 0; local < 4; ++local)
                        {
                            cell.vertex_ids[local] = previous_global_ids[
                                static_cast<std::size_t>(
                                    bottom.vertex_ids[local])];
                            cell.vertex_ids[local + 4] = next_global_ids[
                                static_cast<std::size_t>(
                                    top->vertex_ids[local])];
                        }
                        cells.push_back(cell);
                    }
                    return true;
                },
                previous_face);
        }

        std::vector<VertexId> faceVertexIds(
            const SurfaceFace &face)
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

        struct ContinuingFront
        {
            GrowthFront front;
            std::vector<VertexId> global_vertex_ids;
        };

        Result<ContinuingFront, InvalidFaceConstraintState>
        buildContinuingFront(
            const LayerStepResult &step,
            const std::vector<VertexId> &global_vertex_ids,
            const FaceLayerConstraintTable &constraints)
        {
            using BuildResult =
                Result<ContinuingFront, InvalidFaceConstraintState>;
            if (step.next_front.vertices.size() !=
                    global_vertex_ids.size() ||
                step.next_front.faces.size() !=
                    step.next_front.source_face_ids.size())
            {
                return BuildResult::failure({0, step.layer});
            }

            std::vector<bool> keep(
                step.next_front.faces.size(), false);
            std::vector<bool> used(
                step.next_front.vertices.size(), false);
            for (std::size_t face_index = 0;
                 face_index < step.next_front.faces.size();
                 ++face_index)
            {
                const SurfaceFaceId source_face_id =
                    step.next_front.source_face_ids[face_index];
                const FaceLayerConstraint *constraint =
                    constraints.find(source_face_id);
                if (constraint == nullptr)
                {
                    return BuildResult::failure(
                        {source_face_id, step.layer});
                }
                keep[face_index] =
                    constraint->allowed_layer_count > step.layer;
                if (!keep[face_index])
                {
                    continue;
                }
                for (const VertexId vertex_id :
                     faceVertexIds(step.next_front.faces[face_index]))
                {
                    const std::size_t vertex_index =
                        static_cast<std::size_t>(vertex_id);
                    if (vertex_index >= used.size())
                    {
                        return BuildResult::failure(
                            {source_face_id, step.layer});
                    }
                    used[vertex_index] = true;
                }
            }

            ContinuingFront output;
            output.front.layer = step.layer;
            std::vector<VertexId> remap(
                step.next_front.vertices.size(), VertexId{});
            for (std::size_t vertex_index = 0;
                 vertex_index < step.next_front.vertices.size();
                 ++vertex_index)
            {
                if (!used[vertex_index])
                {
                    continue;
                }
                remap[vertex_index] = static_cast<VertexId>(
                    output.front.vertices.size());
                output.front.vertices.push_back(
                    step.next_front.vertices[vertex_index]);
                output.global_vertex_ids.push_back(
                    global_vertex_ids[vertex_index]);
            }

            for (std::size_t face_index = 0;
                 face_index < step.next_front.faces.size();
                 ++face_index)
            {
                if (!keep[face_index])
                {
                    continue;
                }
                output.front.faces.push_back(std::visit(
                    [&](const auto &value) -> SurfaceFace
                    {
                        auto face = value;
                        for (VertexId &vertex_id : face.vertex_ids)
                        {
                            vertex_id = remap[
                                static_cast<std::size_t>(vertex_id)];
                        }
                        return face;
                    },
                    step.next_front.faces[face_index]));
                output.front.source_face_ids.push_back(
                    step.next_front.source_face_ids[face_index]);
            }
            return BuildResult::success(std::move(output));
        }
    }

    Result<RegularLayerGrowthResult, RegularLayerGrowthError>
    RegularLayerGenerator::generate(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options) const
    {
        using GrowthResult =
            Result<RegularLayerGrowthResult, RegularLayerGrowthError>;

        if (!validInitialMapping(patch, initial_front))
        {
            return GrowthResult::failure(
                InvalidLayerFrontMapping{initial_front.layer});
        }

        const auto original_collision =
            buildOriginalSurfaceCollisionIndex(surface_mesh, topology);
        if (!original_collision.hasValue())
        {
            return GrowthResult::failure(
                CollisionInitializationFailure{
                    original_collision.error()});
        }

        const auto profile_result = GrowthProfileBuilder{}.build(
            patch, profiles);
        if (!profile_result.hasValue())
        {
            return GrowthResult::failure(
                GrowthProfileFailure{profile_result.error()});
        }
        const GrowthProfileTable &profile_table = profile_result.value();
        const auto constraint_result = buildFaceLayerConstraints(
            patch, initial_front, profile_table);
        if (!constraint_result.hasValue())
        {
            return GrowthResult::failure(constraint_result.error());
        }
        FaceLayerConstraintTable constraints =
            constraint_result.value();
        const auto propagator_result = TerminationPropagator::build(
            patch, topology);
        if (!propagator_result.hasValue())
        {
            return GrowthResult::failure(propagator_result.error());
        }
        const TerminationPropagator &propagator =
            propagator_result.value();
        const auto initial_propagation = propagator.propagateInitial(
            constraints, options.max_neighbor_layer_difference);
        if (!initial_propagation.hasValue())
        {
            return GrowthResult::failure(initial_propagation.error());
        }

        RegularLayerGrowthResult result;
        GrowthFront current_front = initial_front;
        std::vector<SurfaceFaceId> pending_stop_cells;
        result.mesh.vertices.reserve(initial_front.vertices.size());
        for (const GrowthFrontVertex &vertex : initial_front.vertices)
        {
            result.mesh.vertices.push_back(vertex.position);
        }
        std::vector<VertexId> current_global_ids;
        current_global_ids.reserve(current_front.vertices.size());
        for (const PatchVertex &patch_vertex : patch.vertices())
        {
            const VertexGrowthProfile *profile = profile_table.find(
                patch_vertex.source_vertex_id);
            if (profile == nullptr)
            {
                return GrowthResult::failure(
                    InvalidLayerFrontMapping{0});
            }
            result.layer_vertices.push_back(
                LayerVertexRecord{patch_vertex.source_vertex_id, {}});
            result.vertices.push_back(
                VertexGrowthRecord{
                    patch_vertex.source_vertex_id,
                    *profile,
                    0});
        }
        for (std::size_t index = 0;
             index < initial_front.vertices.size();
             ++index)
        {
            if (index > static_cast<std::size_t>(
                            std::numeric_limits<VertexId>::max()))
            {
                return GrowthResult::failure(
                    VolumeVertexIdOverflow{index});
            }
            const VertexId global_id = static_cast<VertexId>(index);
            current_global_ids.push_back(global_id);
            LayerVertexRecord *record = findLayerRecord(
                result.layer_vertices,
                initial_front.vertices[index].source_vertex_id);
            if (record == nullptr)
            {
                return GrowthResult::failure(
                    InvalidLayerFrontMapping{0});
            }
            record->layer_vertex_ids.push_back(global_id);
        }
        for (const SurfaceFaceId source_face_id :
             patch.sourceFaceIds())
        {
            result.faces.push_back(
                FaceGrowthRecord{source_face_id});
        }

        ExposedBoundaryTracker exposed_boundary;
        while (!current_front.faces.empty())
        {
            const std::uint32_t target_layer =
                current_front.layer + 1;
            const bool report_layer =
                hasLayerAttempt(current_front, constraints);
            if (report_layer)
            {
                std::cout
                    << "generate "
                    << target_layer
                    << " boundarylayer"
                    << std::endl;
            }

            const auto step_result = RegularLayerStepper{}.step(
                current_front, profile_table, constraints, options);
            if (!step_result.hasValue())
            {
                return GrowthResult::failure(step_result.error());
            }
            result.smoothing_diagnostics.activated_vertices +=
                step_result.value().smoothing_diagnostics.activated_vertices;
            result.smoothing_diagnostics.updated_vertices +=
                step_result.value().smoothing_diagnostics.updated_vertices;
            result.smoothing_diagnostics.maximum_skewness_before =
                std::max(
                    result.smoothing_diagnostics.maximum_skewness_before,
                    step_result.value().smoothing_diagnostics
                        .maximum_skewness_before);
            result.smoothing_diagnostics.maximum_skewness_after =
                std::max(
                    result.smoothing_diagnostics.maximum_skewness_after,
                    step_result.value().smoothing_diagnostics
                        .maximum_skewness_after);
            const auto quality_propagation = propagator.applyDirectStops(
                constraints,
                directQualityStops(step_result.value().stopped_faces),
                options.max_neighbor_layer_difference);
            if (!quality_propagation.hasValue())
            {
                return GrowthResult::failure(quality_propagation.error());
            }
            const auto quality_step = propagator.filterCandidates(
                current_front,
                step_result.value(),
                constraints);
            if (!quality_step.hasValue())
            {
                return GrowthResult::failure(quality_step.error());
            }
            const auto isotropic_propagation =
                propagator.applyDirectStops(
                    constraints,
                    quality_step.value().accepted_stopped_faces,
                    options.max_neighbor_layer_difference);
            if (!isotropic_propagation.hasValue())
            {
                return GrowthResult::failure(
                    isotropic_propagation.error());
            }
            const auto isotropic_step = propagator.filterCandidates(
                current_front,
                quality_step.value(),
                constraints);
            if (!isotropic_step.hasValue())
            {
                return GrowthResult::failure(isotropic_step.error());
            }
            const auto obstacle_step =
                LayerCollisionChecker{}.filterAgainstObstacles(
                    original_collision.value(),
                    exposed_boundary,
                    current_front,
                    isotropic_step.value());
            if (!obstacle_step.hasValue())
            {
                return GrowthResult::failure(
                    CollisionStateFailure{
                        step_result.value().layer,
                        obstacle_step.error()});
            }
            const auto obstacle_propagation = propagator.applyDirectStops(
                constraints,
                addedCollisionStops(
                    obstacle_step.value().stopped_faces,
                    isotropic_step.value().stopped_faces),
                options.max_neighbor_layer_difference);
            if (!obstacle_propagation.hasValue())
            {
                return GrowthResult::failure(obstacle_propagation.error());
            }
            const auto propagated_obstacle_step =
                propagator.filterCandidates(
                    current_front,
                    obstacle_step.value(),
                    constraints);
            if (!propagated_obstacle_step.hasValue())
            {
                return GrowthResult::failure(
                    propagated_obstacle_step.error());
            }
            const auto collision_step =
                LayerCollisionChecker{}.filterSelfCollisions(
                    current_front,
                    propagated_obstacle_step.value());
            if (!collision_step.hasValue())
            {
                return GrowthResult::failure(
                    CollisionStateFailure{
                        step_result.value().layer,
                        collision_step.error()});
            }
            const auto self_propagation = propagator.applyDirectStops(
                constraints,
                addedCollisionStops(
                    collision_step.value().stopped_faces,
                    propagated_obstacle_step.value().stopped_faces),
                options.max_neighbor_layer_difference);
            if (!self_propagation.hasValue())
            {
                return GrowthResult::failure(self_propagation.error());
            }
            const auto final_step = options.enforce_single_high_edge
                ? propagator.filterSingleHighEdgeCandidates(
                      current_front,
                      collision_step.value(),
                      constraints,
                      options.max_neighbor_layer_difference,
                      pending_stop_cells)
                : propagator.filterCandidates(
                      current_front,
                      collision_step.value(),
                      constraints);
            if (!final_step.hasValue())
            {
                return GrowthResult::failure(final_step.error());
            }
            const LayerStepResult &step = final_step.value();
            if (step.next_front.vertices.size() !=
                    step.previous_front_vertex_indices.size() ||
                step.next_front.faces.size() !=
                    step.previous_front_face_indices.size())
            {
                return GrowthResult::failure(
                    InvalidLayerFrontMapping{current_front.layer});
            }

            const std::size_t first_new_index = result.mesh.vertices.size();
            const std::size_t new_count = step.next_front.vertices.size();
            if (new_count > 0 &&
                (first_new_index >
                     std::numeric_limits<std::size_t>::max() - new_count ||
                 first_new_index + new_count - 1 >
                     static_cast<std::size_t>(
                         std::numeric_limits<VertexId>::max())))
            {
                return GrowthResult::failure(
                    VolumeVertexIdOverflow{
                        first_new_index + new_count - 1});
            }

            std::vector<VertexId> next_global_ids;
            next_global_ids.reserve(new_count);
            for (std::size_t index = 0; index < new_count; ++index)
            {
                next_global_ids.push_back(static_cast<VertexId>(
                    first_new_index + index));
            }

            std::vector<VolumeCell> new_cells;
            std::vector<CellMetadata> new_metadata;
            new_cells.reserve(step.next_front.faces.size());
            new_metadata.reserve(step.next_front.faces.size());
            for (std::size_t next_face_index = 0;
                 next_face_index < step.next_front.faces.size();
                 ++next_face_index)
            {
                const std::size_t previous_face_index =
                    step.previous_front_face_indices[next_face_index];
                if (previous_face_index >= current_front.faces.size() ||
                    !appendCandidateCell(
                        current_front.faces[previous_face_index],
                        step.next_front.faces[next_face_index],
                        current_global_ids,
                        next_global_ids,
                        new_cells))
                {
                    return GrowthResult::failure(
                        InvalidLayerFrontMapping{current_front.layer});
                }
                new_metadata.push_back(
                    CellMetadata{
                        CellRole::RegularLayer,
                        step.next_front.source_face_ids[next_face_index],
                        step.layer});
            }

            auto boundary_candidates = buildLayerBoundaryCandidates(
                current_front, step);
            if (!boundary_candidates.hasValue())
            {
                return GrowthResult::failure(
                    CollisionStateFailure{
                        step.layer,
                        boundary_candidates.error()});
            }
            for (LayerBoundaryCandidate &candidate :
                 boundary_candidates.value())
            {
                const std::size_t source_index =
                    static_cast<std::size_t>(candidate.top.source_face_id);
                if (source_index >= surface_mesh.face_tags.size())
                {
                    return GrowthResult::failure(
                        CollisionStateFailure{
                            step.layer,
                            SpatialError::InvalidTopologyReference});
                }
                const std::uint32_t region_id =
                    surface_mesh.face_tags[source_index].region_id;
                candidate.bottom.region_id = region_id;
                candidate.top.region_id = region_id;
            }
            const auto boundary_update = exposed_boundary.prepare(
                boundary_candidates.value());
            if (!boundary_update.hasValue())
            {
                return GrowthResult::failure(
                    CollisionStateFailure{
                        step.layer,
                        boundary_update.error()});
            }

            for (const GrowthFrontVertex &vertex : step.next_front.vertices)
            {
                result.mesh.vertices.push_back(vertex.position);
            }
            result.mesh.cells.insert(
                result.mesh.cells.end(),
                new_cells.begin(),
                new_cells.end());
            result.mesh.metadata.insert(
                result.mesh.metadata.end(),
                new_metadata.begin(),
                new_metadata.end());
            exposed_boundary.apply(boundary_update.value());

            if (report_layer)
            {
                std::cout
                    << "finish "
                    << step.layer
                    << " boundarylayer. add "
                    << new_cells.size()
                    << " cell"
                    << std::endl;
            }

            for (std::size_t index = 0;
                 index < step.next_front.vertices.size();
                 ++index)
            {
                const VertexId source_id =
                    step.next_front.vertices[index].source_vertex_id;
                LayerVertexRecord *layer_record = findLayerRecord(
                    result.layer_vertices, source_id);
                VertexGrowthRecord *vertex_record = findVertexRecord(
                    result.vertices, source_id);
                if (layer_record == nullptr || vertex_record == nullptr)
                {
                    return GrowthResult::failure(
                        InvalidLayerFrontMapping{current_front.layer});
                }
                layer_record->layer_vertex_ids.push_back(
                    next_global_ids[index]);
                vertex_record->accepted_layer_count = step.layer;
            }
            for (const SurfaceFaceId source_face_id :
                 step.next_front.source_face_ids)
            {
                FaceGrowthRecord *record = findFaceRecord(
                    result.faces, source_face_id);
                if (record == nullptr)
                {
                    return GrowthResult::failure(
                        InvalidLayerFrontMapping{current_front.layer});
                }
                record->accepted_layer_count = std::max(
                    record->accepted_layer_count,
                    step.layer);

                const FaceLayerConstraint *constraint =
                    constraints.find(source_face_id);
                if (constraint == nullptr)
                {
                    return GrowthResult::failure(
                        InvalidFaceConstraintState{
                            source_face_id,
                            step.layer});
                }
                if (constraint->allowed_layer_count == step.layer)
                {
                    record->stop_layer = step.layer + 1;
                    if (constraint->limit_kind ==
                        FaceLayerLimitKind::Requested)
                    {
                        record->status = FaceGrowthStatus::Completed;
                        record->stop_reason =
                            FaceStopReason::VertexLayerLimit;
                    }
                    else
                    {
                        record->status = FaceGrowthStatus::Stopped;
                        record->stop_reason =
                            constraint->limit_kind ==
                                    FaceLayerLimitKind::DirectStop
                                ? constraint->direct_reason
                                : FaceStopReason::
                                      NeighborLayerConstraint;
                    }
                }
            }
            for (const FaceStopEvent &event : step.stopped_faces)
            {
                FaceGrowthRecord *record = findFaceRecord(
                    result.faces, event.source_face_id);
                if (record == nullptr)
                {
                    return GrowthResult::failure(
                        InvalidLayerFrontMapping{current_front.layer});
                }
                record->status = FaceGrowthStatus::Stopped;
                record->stop_reason = event.reason;
                record->stop_layer = event.layer;
            }
            for (const FaceStopEvent &event : step.completed_faces)
            {
                FaceGrowthRecord *record = findFaceRecord(
                    result.faces, event.source_face_id);
                if (record == nullptr)
                {
                    return GrowthResult::failure(
                        InvalidLayerFrontMapping{current_front.layer});
                }
                record->status = FaceGrowthStatus::Completed;
                record->stop_reason = event.reason;
                record->stop_layer = event.layer;
            }

            const auto continuing = buildContinuingFront(
                step,
                next_global_ids,
                constraints);
            if (!continuing.hasValue())
            {
                return GrowthResult::failure(continuing.error());
            }
            current_front = continuing.value().front;
            current_global_ids =
                continuing.value().global_vertex_ids;
        }

        // 一层都未接受的 Wall 面不在外露边界跟踪器中，显式补回最终接口。
        std::vector<SurfaceFaceId> zero_layer_source_face_ids;
        for (const FaceGrowthRecord &record : result.faces)
        {
            if (record.accepted_layer_count == 0)
            {
                zero_layer_source_face_ids.push_back(record.source_face_id);
            }
        }
        const auto farfield_boundary = buildFarfieldBoundary(
            surface_mesh,
            initial_front,
            exposed_boundary,
            zero_layer_source_face_ids);
        if (!farfield_boundary.hasValue())
        {
            return GrowthResult::failure(
                CollisionStateFailure{
                    current_front.layer,
                    farfield_boundary.error()});
        }
        result.farfield_boundary = farfield_boundary.value();
        const auto top_surface = extractBoundaryLayerTop(
            result.farfield_boundary);
        if (!top_surface.hasValue())
        {
            return GrowthResult::failure(
                CollisionStateFailure{
                    current_front.layer,
                    top_surface.error()});
        }
        result.top_surface = top_surface.value();

        return GrowthResult::success(std::move(result));
    }

    Result<RegularLayerGrowthResult, RegularLayerGrowthError>
    generateRegularLayers(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options)
    {
        return RegularLayerGenerator{}.generate(
            surface_mesh,
            topology,
            patch,
            initial_front,
            profiles,
            options);
    }
}
