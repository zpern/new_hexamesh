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

        std::vector<std::array<VertexId, 3>> triangularCellFaces(
            const VolumeCell &cell)
        {
            return std::visit(
                [](const auto &value)
                {
                    using Cell = std::decay_t<decltype(value)>;
                    const auto &v = value.vertex_ids;
                    if constexpr (std::is_same_v<Cell, Tetra>)
                        return std::vector<std::array<VertexId, 3>>{
                            {v[0],v[1],v[2]}, {v[0],v[3],v[1]},
                            {v[1],v[3],v[2]}, {v[2],v[3],v[0]}};
                    else if constexpr (std::is_same_v<Cell, Pyramid>)
                        return std::vector<std::array<VertexId, 3>>{
                            {v[0],v[1],v[4]}, {v[1],v[2],v[4]},
                            {v[2],v[3],v[4]}, {v[3],v[0],v[4]}};
                    else if constexpr (std::is_same_v<Cell, Prism>)
                        return std::vector<std::array<VertexId, 3>>{
                            {v[0],v[1],v[2]}, {v[3],v[4],v[5]}};
                    else
                        return std::vector<std::array<VertexId, 3>>{};
                },
                cell);
        }

        const LayerVertexRecord *findLayerVertices(
            const LayerVertexTable &table,
            VertexId source_id)
        {
            const auto found = std::find_if(
                table.begin(), table.end(),
                [source_id](const LayerVertexRecord &entry)
                { return entry.source_vertex_id == source_id; });
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
    }

    std::vector<bool> externallyExposedTopTriangles(
        const std::vector<Triangle> &candidates,
        const std::vector<VolumeCell> &cells)
    {
        std::map<std::array<VertexId, 3>, std::size_t> incidence;
        for (const VolumeCell &cell : cells)
            for (const auto &face : triangularCellFaces(cell))
                ++incidence[triangleKey(face)];

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
        const auto exposed = externallyExposedTopTriangles(
            candidates, output.mesh.cells);
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
}
