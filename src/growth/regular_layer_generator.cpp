#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/growth/farfield_boundary_builder.hpp>
#include <boundary_mesh/growth/layer_collision_checker.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/growth/regular_layer_stepper.hpp>
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
                front.vertices.size() != front.source_vertex_ids.size() ||
                front.vertices.size() != front.vertex_boundaries.size() ||
                front.faces.size() != front.source_face_ids.size() ||
                front.source_vertex_ids.size() != patch.vertices().size() ||
                front.source_face_ids != patch.sourceFaceIds())
            {
                return false;
            }
            for (std::size_t index = 0;
                 index < patch.vertices().size();
                 ++index)
            {
                if (front.source_vertex_ids[index] !=
                    patch.vertices()[index].source_vertex_id)
                {
                    return false;
                }
            }
            return true;
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

        RegularLayerGrowthResult result;
        result.mesh.vertices = initial_front.vertices;
        std::vector<VertexId> current_global_ids;
        current_global_ids.reserve(initial_front.vertices.size());
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
            result.layer_vertices.push_back(
                LayerVertexRecord{
                    initial_front.source_vertex_ids[index],
                    {global_id}});
            const VertexGrowthProfile *profile = profile_table.find(
                initial_front.source_vertex_ids[index]);
            if (profile == nullptr)
            {
                return GrowthResult::failure(
                    InvalidLayerFrontMapping{0});
            }
            result.vertices.push_back(
                VertexGrowthRecord{
                    initial_front.source_vertex_ids[index],
                    *profile,
                    0});
        }
        for (const SurfaceFaceId source_face_id :
             initial_front.source_face_ids)
        {
            result.faces.push_back(
                FaceGrowthRecord{source_face_id});
        }

        GrowthFront current_front = initial_front;
        ExposedBoundaryTracker exposed_boundary;
        while (!current_front.faces.empty())
        {
            const auto step_result = RegularLayerStepper{}.step(
                current_front, profile_table, options);
            if (!step_result.hasValue())
            {
                return GrowthResult::failure(step_result.error());
            }
            const auto obstacle_step =
                LayerCollisionChecker{}.filterAgainstObstacles(
                    original_collision.value(),
                    exposed_boundary,
                    current_front,
                    step_result.value());
            if (!obstacle_step.hasValue())
            {
                return GrowthResult::failure(
                    CollisionStateFailure{
                        step_result.value().layer,
                        obstacle_step.error()});
            }
            const auto collision_step =
                LayerCollisionChecker{}.filterSelfCollisions(
                    current_front,
                    obstacle_step.value());
            if (!collision_step.hasValue())
            {
                return GrowthResult::failure(
                    CollisionStateFailure{
                        step_result.value().layer,
                        collision_step.error()});
            }
            const LayerStepResult &step = collision_step.value();
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

            result.mesh.vertices.insert(
                result.mesh.vertices.end(),
                step.next_front.vertices.begin(),
                step.next_front.vertices.end());
            result.mesh.cells.insert(
                result.mesh.cells.end(),
                new_cells.begin(),
                new_cells.end());
            result.mesh.metadata.insert(
                result.mesh.metadata.end(),
                new_metadata.begin(),
                new_metadata.end());
            exposed_boundary.apply(boundary_update.value());

            for (std::size_t index = 0;
                 index < step.next_front.source_vertex_ids.size();
                 ++index)
            {
                const VertexId source_id =
                    step.next_front.source_vertex_ids[index];
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
                ++record->accepted_layer_count;
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

            current_front = step.next_front;
            current_global_ids = std::move(next_global_ids);
        }

        const auto farfield_boundary = buildFarfieldBoundary(
            surface_mesh, exposed_boundary);
        if (!farfield_boundary.hasValue())
        {
            return GrowthResult::failure(
                CollisionStateFailure{
                    current_front.layer,
                    farfield_boundary.error()});
        }
        result.farfield_boundary = farfield_boundary.value();

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
