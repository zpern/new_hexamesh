#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <map>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/transition/reserved_layer_transition.hpp>

namespace boundary_mesh
{
    namespace
    {
        std::array<VertexId, 3> triangleKey(
            std::array<VertexId, 3> ids)
        {
            std::sort(ids.begin(), ids.end());
            return ids;
        }

        const LayerVertexRecord *findLayerVertices(
            const LayerVertexTable &table,
            VertexId source_id,
            std::uint32_t branch_id = 0)
        {
            const auto found = std::find_if(
                table.begin(), table.end(),
                [source_id, branch_id](const LayerVertexRecord &entry)
                {
                    return entry.source_vertex_id == source_id &&
                           entry.branch_id == branch_id;
                });
            return found == table.end() ? nullptr : &*found;
        }

        const CoordinatedTransitionFace *findFaceState(
            const std::vector<CoordinatedTransitionFace> &faces,
            SurfaceFaceId source_id)
        {
            const auto found = std::lower_bound(
                faces.begin(), faces.end(), source_id,
                [](const CoordinatedTransitionFace &face,
                   SurfaceFaceId value)
                { return face.layers.source_face_id < value; });
            return found == faces.end() ||
                           found->layers.source_face_id != source_id
                       ? nullptr
                       : &*found;
        }

        template <std::size_t N>
        bool buildLayerIds(
            const std::array<VertexId, N> &source_ids,
            std::uint32_t trial_layers,
            const LayerVertexTable &table,
            std::vector<std::array<VertexId, N>> &output)
        {
            output.resize(static_cast<std::size_t>(trial_layers) + 1);
            for (std::size_t corner = 0; corner < N; ++corner)
            {
                const LayerVertexRecord *record = findLayerVertices(
                    table, source_ids[corner]);
                if (record == nullptr ||
                    record->layer_vertex_ids.size() <= trial_layers)
                    return false;
                for (std::uint32_t layer = 0;
                     layer <= trial_layers;
                     ++layer)
                    output[layer][corner] =
                        record->layer_vertex_ids[layer];
            }
            return true;
        }

        template <std::size_t N>
        bool buildFrontLayerIds(
            const std::array<VertexId, N> &front_ids,
            const GrowthFront &front,
            std::uint32_t trial_layers,
            const LayerVertexTable &table,
            std::vector<std::array<VertexId, N>> &output)
        {
            output.resize(static_cast<std::size_t>(trial_layers) + 1);
            for (std::size_t corner = 0; corner < N; ++corner)
            {
                const std::size_t front_index =
                    static_cast<std::size_t>(front_ids[corner]);
                if (front_index >= front.vertices.size()) return false;
                const GrowthFrontVertex &vertex =
                    front.vertices[front_index];
                const LayerVertexRecord *record = findLayerVertices(
                    table,
                    vertex.source_vertex_id,
                    vertex.branch_id);
                if (record == nullptr ||
                    record->layer_vertex_ids.size() <= trial_layers)
                    return false;
                for (std::uint32_t layer = 0;
                     layer <= trial_layers;
                     ++layer)
                    output[layer][corner] =
                        record->layer_vertex_ids[layer];
            }
            return true;
        }

        const FaceGrowthRecord *findGrowthFace(
            const std::vector<FaceGrowthRecord> &faces,
            SurfaceFaceId source_id)
        {
            const auto found = std::find_if(
                faces.begin(), faces.end(),
                [source_id](const FaceGrowthRecord &face)
                { return face.source_face_id == source_id; });
            return found == faces.end() ? nullptr : &*found;
        }

        Result<std::vector<CoordinatedTransitionFace>,
               TransitionCoordinationError>
        coordinateFront(
            const GrowthFront &front,
            const RegularLayerGrowthResult &trial)
        {
            using ResultType = Result<
                std::vector<CoordinatedTransitionFace>,
                TransitionCoordinationError>;
            if (front.faces.size() != front.source_face_ids.size())
                return ResultType::failure(
                    MissingTransitionFaceState{});

            struct EdgeUse
            {
                std::size_t face{};
                std::size_t local_edge{};
            };
            std::map<std::pair<VertexId, VertexId>, std::vector<EdgeUse>>
                edge_uses;
            std::vector<CoordinatedTransitionFace> output(
                front.faces.size());
            for (std::size_t face_index = 0;
                 face_index < front.faces.size();
                 ++face_index)
            {
                const SurfaceFaceId source_id =
                    front.source_face_ids[face_index];
                const FaceGrowthRecord *growth = findGrowthFace(
                    trial.faces, source_id);
                if (growth == nullptr)
                    return ResultType::failure(
                        MissingTransitionFaceState{source_id});
                output[face_index].layers = FaceLayerState{
                    source_id,
                    growth->accepted_layer_count,
                    occupiedLayerCount(growth->accepted_layer_count),
                    regularLayerCount(growth->accepted_layer_count)};
                std::visit(
                    [&](const auto &face)
                    {
                        for (std::size_t edge = 0;
                             edge < face.vertex_ids.size();
                             ++edge)
                        {
                            VertexId first = face.vertex_ids[edge];
                            VertexId second = face.vertex_ids[
                                (edge + 1) % face.vertex_ids.size()];
                            if (second < first) std::swap(first, second);
                            edge_uses[{first, second}].push_back(
                                EdgeUse{face_index, edge});
                        }
                    },
                    front.faces[face_index]);
            }

            std::vector<std::vector<std::size_t>> high_edges(
                front.faces.size());
            for (const auto &[edge, uses] : edge_uses)
            {
                (void)edge;
                if (uses.size() != 2) continue;
                const auto &first = output[uses[0].face].layers;
                const auto &second = output[uses[1].face].layers;
                const std::uint32_t low = std::min(
                    first.occupied_layers, second.occupied_layers);
                const std::uint32_t high = std::max(
                    first.occupied_layers, second.occupied_layers);
                if (high > low + 1)
                    return ResultType::failure(
                        UncoordinatedTransitionLayerDifference{
                            first.source_face_id,
                            second.source_face_id});
                if (first.occupied_layers + 1 ==
                    second.occupied_layers)
                    high_edges[uses[0].face].push_back(
                        uses[0].local_edge);
                if (second.occupied_layers + 1 ==
                    first.occupied_layers)
                    high_edges[uses[1].face].push_back(
                        uses[1].local_edge);
            }
            for (std::size_t face_index = 0;
                 face_index < output.size();
                 ++face_index)
            {
                auto &edges = high_edges[face_index];
                const bool quad = std::holds_alternative<Quad>(
                    front.faces[face_index]);
                if (edges.size() > (quad ? 2u : 1u))
                    return ResultType::failure(
                        MultipleTransitionHighEdges{
                            output[face_index].layers.source_face_id,
                            edges});
                if (edges.size() == 2 &&
                    (edges[0] + 1) % 4 != edges[1] &&
                    (edges[1] + 1) % 4 != edges[0])
                    return ResultType::failure(
                        MultipleTransitionHighEdges{
                            output[face_index].layers.source_face_id,
                            edges});
                if (!edges.empty())
                    output[face_index].high_edge_local_index = edges[0];
                if (edges.size() == 2)
                    output[face_index].second_high_edge_local_index =
                        edges[1];
            }
            return ResultType::success(std::move(output));
        }

        bool remapTopForMultiNormalMerge(
            SurfaceMesh &top,
            const MultiNormalTransitionResult &multi_normal,
            std::size_t reserved_vertex_count)
        {
            if (!multi_normal.applied)
                return true;
            const std::size_t interface_count =
                multi_normal.transformed_front.vertices.size();
            if (multi_normal.transformed_front_volume_vertex_ids.size() !=
                    interface_count ||
                reserved_vertex_count < interface_count)
                return false;
            const std::size_t transition_vertex_count =
                multi_normal.transition_cells.vertices.size();
            bool valid = true;
            for (SurfaceFace &face : top.faces)
            {
                std::visit(
                    [&](auto &typed_face)
                    {
                        for (VertexId &id : typed_face.vertex_ids)
                        {
                            const std::size_t index =
                                static_cast<std::size_t>(id);
                            if (index >= reserved_vertex_count)
                            {
                                valid = false;
                                return;
                            }
                            if (index < interface_count)
                                id = multi_normal
                                    .transformed_front_volume_vertex_ids[index];
                            else
                                id = static_cast<VertexId>(
                                    transition_vertex_count +
                                    index - interface_count);
                        }
                    },
                    face);
                if (!valid) return false;
            }
            return true;
        }

        SurfaceMesh combineFarfieldAndTop(
            const SurfaceMesh &original,
            const SurfaceMesh &top,
            const std::vector<Point3> &volume_vertices)
        {
            SurfaceMesh output;
            const auto appendFace = [&output](
                const SurfaceFace &face,
                const SurfaceBoundaryTag &tag,
                const std::vector<Point3> &points)
            {
                SurfaceFace remapped = face;
                std::visit(
                    [&](auto &typed_face)
                    {
                        for (VertexId &id : typed_face.vertex_ids)
                        {
                            const std::size_t index =
                                static_cast<std::size_t>(id);
                            if (index >= points.size()) return;
                            id = static_cast<VertexId>(
                                output.vertices.size());
                            output.vertices.push_back(points[index]);
                        }
                    },
                    remapped);
                output.faces.push_back(std::move(remapped));
                output.face_tags.push_back(tag);
            };
            for (std::size_t index = 0;
                 index < original.faces.size();
                 ++index)
                if (index < original.face_tags.size() &&
                    original.face_tags[index].kind ==
                        SurfaceBoundaryKind::Farfield)
                    appendFace(
                        original.faces[index],
                        original.face_tags[index],
                        original.vertices);
            for (std::size_t index = 0;
                 index < top.faces.size();
                 ++index)
                appendFace(
                    top.faces[index],
                    top.face_tags[index],
                    volume_vertices);
            return output;
        }
    }

    std::vector<bool> nonDuplicatedTopTriangles(
        const std::vector<Triangle> &candidates)
    {
        std::map<std::array<VertexId, 3>, std::size_t> incidence;
        for (const Triangle &candidate : candidates)
            ++incidence[triangleKey(candidate.vertex_ids)];

        std::vector<bool> exposed(candidates.size(), false);
        for (std::size_t candidate_index = 0;
             candidate_index < candidates.size();
             ++candidate_index)
        {
            const auto key = triangleKey(
                candidates[candidate_index].vertex_ids);
            const auto found = incidence.find(key);
            exposed[candidate_index] =
                found != incidence.end() && found->second == 1;
        }
        return exposed;
    }

    Result<ReservedLayerTransitionResult, ReservedLayerTransitionError>
    generateReservedLayerTransition(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options)
    {
        using PipelineResult = Result<
            ReservedLayerTransitionResult,
            ReservedLayerTransitionError>;

        const auto trial_profiles = makeReservedTrialProfiles(profiles);
        if (!trial_profiles.hasValue())
            return PipelineResult::failure(
                ReservedLayerTransitionError{trial_profiles.error()});

        RegularLayerGrowthOptions trial_options = options;
        trial_options.enforce_single_high_edge = true;
        auto trial = generateRegularLayers(
            surface_mesh, topology, patch, initial_front,
            trial_profiles.value(), trial_options);
        if (!trial.hasValue())
            return PipelineResult::failure(
                ReservedLayerTransitionError{trial.error()});

        std::vector<std::pair<SurfaceFaceId, std::uint32_t>> counts;
        for (const FaceGrowthRecord &face : trial.value().faces)
            counts.emplace_back(
                face.source_face_id, face.accepted_layer_count);
        const auto coordinated = TransitionLayerCoordinator{}.coordinate(
            patch, topology, counts);
        if (!coordinated.hasValue())
            return PipelineResult::failure(
                ReservedLayerTransitionError{coordinated.error()});

        ReservedLayerTransitionResult output;
        output.trial_growth = std::move(trial.value());
        output.coordinated_faces = coordinated.value();
        output.mesh.vertices = output.trial_growth.mesh.vertices;

        for (const SurfaceFaceId source_id : patch.sourceFaceIds())
        {
            const CoordinatedTransitionFace *state = findFaceState(
                output.coordinated_faces, source_id);
            if (state == nullptr ||
                static_cast<std::size_t>(source_id) >=
                    surface_mesh.faces.size())
                return PipelineResult::failure(
                    ReservedLayerTransitionError{
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{source_id}}});

            TransitionTemplateResult local = std::visit(
                [&](const auto &source_face) -> TransitionTemplateResult
                {
                    using Face = std::decay_t<decltype(source_face)>;
                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        TriangleTransitionInput input;
                        input.source_face_id = source_id;
                        input.trial_layers = state->layers.trial_layers;
                        input.high_edge_local_index =
                            state->high_edge_local_index;
                        if (!buildLayerIds(
                                source_face.vertex_ids,
                                input.trial_layers,
                                output.trial_growth.layer_vertices,
                                input.layer_vertex_ids))
                            return TransitionTemplateResult::failure(
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        source_id}});
                        return buildTriangleTransition(input);
                    }
                    else
                    {
                        if (output.mesh.vertices.size() >
                            static_cast<std::size_t>(
                                std::numeric_limits<VertexId>::max()))
                            return TransitionTemplateResult::failure(
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        source_id}});
                        QuadTransitionInput input;
                        input.source_face_id = source_id;
                        input.trial_layers = state->layers.trial_layers;
                        input.high_edge_local_index =
                            state->high_edge_local_index;
                        input.second_high_edge_local_index =
                            state->second_high_edge_local_index;
                        input.mesh_vertices = &output.mesh.vertices;
                        input.center_vertex_id = static_cast<VertexId>(
                            output.mesh.vertices.size());
                        if (!buildLayerIds(
                                source_face.vertex_ids,
                                input.trial_layers,
                                output.trial_growth.layer_vertices,
                                input.layer_vertex_ids))
                            return TransitionTemplateResult::failure(
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        source_id}});
                        return buildQuadTransition(input);
                    }
                },
                surface_mesh.faces[source_id]);
            if (!local.hasValue())
                return PipelineResult::failure(
                    ReservedLayerTransitionError{local.error()});

            output.mesh.vertices.insert(
                output.mesh.vertices.end(),
                local.value().created_vertices.begin(),
                local.value().created_vertices.end());
            output.mesh.cells.insert(
                output.mesh.cells.end(),
                local.value().volume_cells.begin(),
                local.value().volume_cells.end());
            output.mesh.metadata.insert(
                output.mesh.metadata.end(),
                local.value().metadata.begin(),
                local.value().metadata.end());
            for (const Triangle &face : local.value().top_faces)
            {
                output.boundary_layer_top.faces.push_back(face);
                output.boundary_layer_top.face_tags.push_back(
                    SurfaceBoundaryTag{
                        SurfaceBoundaryKind::BoundaryLayerInterface,
                        surface_mesh.face_tags[source_id].region_id});
            }
        }
        std::vector<Triangle> candidates;
        candidates.reserve(output.boundary_layer_top.faces.size());
        for (const SurfaceFace &face : output.boundary_layer_top.faces)
            candidates.push_back(std::get<Triangle>(face));
        const auto exposed = nonDuplicatedTopTriangles(candidates);
        SurfaceMesh filtered_top;
        filtered_top.vertices = output.mesh.vertices;
        for (std::size_t index = 0; index < candidates.size(); ++index)
        {
            if (!exposed[index]) continue;
            filtered_top.faces.push_back(candidates[index]);
            filtered_top.face_tags.push_back(
                output.boundary_layer_top.face_tags[index]);
        }
        output.boundary_layer_top = std::move(filtered_top);
        return PipelineResult::success(std::move(output));
    }

    Result<ReservedLayerTransitionResult,
           CombinedReservedLayerTransitionError>
    generateReservedLayerTransition(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const MultiNormalOptions &multi_normal_options,
        const RegularLayerGrowthOptions &options)
    {
        using PipelineResult = Result<
            ReservedLayerTransitionResult,
            CombinedReservedLayerTransitionError>;

        auto multi_normal = generateMultiNormalTransition(
            initial_front, multi_normal_options);
        if (!multi_normal.hasValue())
            return PipelineResult::failure(
                ReservedMultiNormalFailure{multi_normal.error()});

        const auto trial_profiles = makeReservedTrialProfiles(profiles);
        if (!trial_profiles.hasValue())
            return PipelineResult::failure(trial_profiles.error());
        RegularLayerGrowthOptions trial_options = options;
        trial_options.enforce_single_high_edge = true;
        auto trial = generateRegularLayers(
            surface_mesh,
            topology,
            patch,
            multi_normal.value().transformed_front,
            trial_profiles.value(),
            trial_options);
        if (!trial.hasValue())
            return PipelineResult::failure(trial.error());

        const auto coordinated = coordinateFront(
            multi_normal.value().transformed_front,
            trial.value());
        if (!coordinated.hasValue())
            return PipelineResult::failure(coordinated.error());

        ReservedLayerTransitionResult output;
        output.trial_growth = std::move(trial.value());
        output.coordinated_faces = coordinated.value();
        output.mesh.vertices = output.trial_growth.mesh.vertices;
        const GrowthFront &front =
            multi_normal.value().transformed_front;
        for (std::size_t face_index = 0;
             face_index < front.faces.size();
             ++face_index)
        {
            const SurfaceFaceId source_id =
                front.source_face_ids[face_index];
            if (static_cast<std::size_t>(source_id) >=
                surface_mesh.face_tags.size())
                return PipelineResult::failure(
                    TransitionTemplateError{
                        InvalidTransitionTemplateInput{source_id}});
            const CoordinatedTransitionFace &state =
                output.coordinated_faces[face_index];
            TransitionTemplateResult local = std::visit(
                [&](const auto &face) -> TransitionTemplateResult
                {
                    using Face = std::decay_t<decltype(face)>;
                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        TriangleTransitionInput input;
                        input.source_face_id = source_id;
                        input.trial_layers = state.layers.trial_layers;
                        input.high_edge_local_index =
                            state.high_edge_local_index;
                        if (!buildFrontLayerIds(
                                face.vertex_ids,
                                front,
                                input.trial_layers,
                                output.trial_growth.layer_vertices,
                                input.layer_vertex_ids))
                            return TransitionTemplateResult::failure(
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        source_id}});
                        return buildTriangleTransition(input);
                    }
                    else
                    {
                        if (output.mesh.vertices.size() >
                            static_cast<std::size_t>(
                                std::numeric_limits<VertexId>::max()))
                            return TransitionTemplateResult::failure(
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        source_id}});
                        QuadTransitionInput input;
                        input.source_face_id = source_id;
                        input.trial_layers = state.layers.trial_layers;
                        input.high_edge_local_index =
                            state.high_edge_local_index;
                        input.second_high_edge_local_index =
                            state.second_high_edge_local_index;
                        input.mesh_vertices = &output.mesh.vertices;
                        input.center_vertex_id = static_cast<VertexId>(
                            output.mesh.vertices.size());
                        if (!buildFrontLayerIds(
                                face.vertex_ids,
                                front,
                                input.trial_layers,
                                output.trial_growth.layer_vertices,
                                input.layer_vertex_ids))
                            return TransitionTemplateResult::failure(
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        source_id}});
                        return buildQuadTransition(input);
                    }
                },
                front.faces[face_index]);
            if (!local.hasValue())
                return PipelineResult::failure(local.error());
            output.mesh.vertices.insert(
                output.mesh.vertices.end(),
                local.value().created_vertices.begin(),
                local.value().created_vertices.end());
            output.mesh.cells.insert(
                output.mesh.cells.end(),
                local.value().volume_cells.begin(),
                local.value().volume_cells.end());
            output.mesh.metadata.insert(
                output.mesh.metadata.end(),
                local.value().metadata.begin(),
                local.value().metadata.end());
            for (const Triangle &face : local.value().top_faces)
            {
                output.boundary_layer_top.faces.push_back(face);
                output.boundary_layer_top.face_tags.push_back(
                    {SurfaceBoundaryKind::BoundaryLayerInterface,
                     surface_mesh.face_tags[source_id].region_id});
            }
        }

        std::vector<Triangle> candidates;
        candidates.reserve(output.boundary_layer_top.faces.size());
        for (const SurfaceFace &face : output.boundary_layer_top.faces)
            candidates.push_back(std::get<Triangle>(face));
        const auto exposed = nonDuplicatedTopTriangles(candidates);
        SurfaceMesh filtered_top;
        filtered_top.vertices = output.mesh.vertices;
        for (std::size_t index = 0; index < candidates.size(); ++index)
        {
            if (!exposed[index]) continue;
            filtered_top.faces.push_back(candidates[index]);
            filtered_top.face_tags.push_back(
                output.boundary_layer_top.face_tags[index]);
        }
        output.boundary_layer_top = std::move(filtered_top);
        output.reserved_transition_cell_count = std::count_if(
            output.mesh.metadata.begin(),
            output.mesh.metadata.end(),
            [](const CellMetadata &metadata)
            { return metadata.role == CellRole::Transition; });
        output.regular_cell_count = output.mesh.cells.size() -
            output.reserved_transition_cell_count;
        const std::size_t reserved_vertex_count = output.mesh.vertices.size();
        if (!remapTopForMultiNormalMerge(
                output.boundary_layer_top,
                multi_normal.value(),
                reserved_vertex_count))
            return PipelineResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{}});
        auto merged = mergeMultiNormalAndRegularMeshes(
            multi_normal.value(), output.mesh);
        if (!merged.hasValue())
            return PipelineResult::failure(
                ReservedMultiNormalMergeFailure{merged.error()});
        output.mesh = std::move(merged.value());
        output.boundary_layer_top.vertices = output.mesh.vertices;
        output.farfield_boundary = combineFarfieldAndTop(
            surface_mesh,
            output.boundary_layer_top,
            output.mesh.vertices);
        output.multi_normal_transition = std::move(multi_normal.value());
        return PipelineResult::success(std::move(output));
    }
}
