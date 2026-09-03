#include <cassert>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/multi_normal/multi_normal_types.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/transition/reserved_layer_transition.hpp>

using namespace boundary_mesh;

namespace
{
    RegularLayerGrowthResult trialWithLayers(
        const std::vector<std::uint32_t> &layers)
    {
        RegularLayerGrowthResult trial;
        for (std::size_t index = 0; index < layers.size(); ++index)
            trial.faces.push_back(FaceGrowthRecord{
                static_cast<SurfaceFaceId>(index), layers[index]});
        return trial;
    }
}

int main()
{
    GrowthFront coordinated_front;
    coordinated_front.vertices.resize(18);
    coordinated_front.faces = {
        Quad{{0,1,2,3}},
        Triangle{{0,4,5}},
        Triangle{{1,6,7}},
        Triangle{{8,9,10}},
        Triangle{{8,11,12}},
        Triangle{{9,13,14}},
        Quad{{0,8,15,16}},
        Triangle{{15,5,17}}};
    coordinated_front.source_face_ids = {0,1,2,3,4,5,6,7};
    const auto coordinated = coordinateTransitionFront(
        coordinated_front,
        trialWithLayers({1,2,2,1,2,2,0,2}));
    assert(coordinated.hasValue());
    assert(!coordinated.value()[0].high_edge_local_index.has_value());
    assert(coordinated.value()[0].continuing_edge_local_index == 0);
    assert(!coordinated.value()[3].high_edge_local_index.has_value());
    assert(coordinated.value()[3].continuing_edge_local_index == 0);
    assert(!coordinated.value()[6].continuing_edge_local_index.has_value());

    GrowthFront opposite_front;
    opposite_front.vertices.resize(8);
    opposite_front.faces = {
        Quad{{0,1,2,3}},
        Triangle{{0,4,5}},
        Triangle{{2,6,7}}};
    opposite_front.source_face_ids = {0,1,2};
    const auto opposite = coordinateTransitionFront(
        opposite_front, trialWithLayers({1,2,2}));
    assert(opposite.hasValue());
    assert(!opposite.value()[0].continuing_edge_local_index.has_value());

    GrowthFront zero_difference_front;
    zero_difference_front.vertices.resize(4);
    zero_difference_front.faces = {
        Triangle{{0,1,2}}, Triangle{{1,0,3}}};
    zero_difference_front.source_face_ids = {0,1};
    const auto zero_difference = coordinateTransitionFront(
        zero_difference_front, trialWithLayers({0,3}));
    assert(!zero_difference.hasValue());
    assert(std::holds_alternative<UncoordinatedTransitionLayerDifference>(
        zero_difference.error()));

    const std::vector<Triangle> candidates{
        Triangle{{0, 1, 2}}, Triangle{{2, 1, 0}},
        Triangle{{0, 1, 3}},
        Triangle{{5, 6, 7}}};
    assert((nonDuplicatedTopTriangles(candidates) ==
            std::vector<bool>{false, false, true, true}));

    SurfaceMesh mesh;
    mesh.vertices = {
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}};
    mesh.faces = {
        Quad{{0,3,2,1}}, Quad{{4,5,6,7}},
        Quad{{0,1,5,4}}, Quad{{1,2,6,5}},
        Quad{{2,3,7,6}}, Quad{{3,0,4,7}}};
    mesh.face_tags.resize(6, {SurfaceBoundaryKind::Farfield, 0});
    mesh.face_tags[1] = {SurfaceBoundaryKind::Wall, 1};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    assert(front.hasValue());
    std::vector<SourceVertexGrowthProfile> profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
        profiles.push_back({vertex.source_vertex_id, {0.05, 1.0, 1}});

    RegularLayerGrowthOptions options;
    options.isotropic_height = 100.0;
    MultiNormalOptions disabled_multi_normal;
    disabled_multi_normal.enabled = false;
    const auto result = generateReservedLayerTransition(
        mesh, topology.value(), patch.value(), front.value(),
        profiles, disabled_multi_normal, options);
    assert(result.hasValue());
    assert(result.value().trial_growth.faces[0].accepted_layer_count == 3);
    assert(result.value().mesh.cells.size() == 8);
    assert(result.value().mesh.metadata.size() == 8);
    assert(result.value().boundary_layer_top.faces.size() == 2);
    for (const SurfaceFace &face : result.value().boundary_layer_top.faces)
        assert(std::holds_alternative<Triangle>(face));
    for (const CellMetadata &metadata : result.value().mesh.metadata)
        assert(metadata.layer <= 2);

    MultiNormalOptions flat_multi_normal;
    flat_multi_normal.enabled = true;
    flat_multi_normal.transition_height = 0.1;
    const auto flat_combined = generateReservedLayerTransition(
        mesh,
        topology.value(),
        patch.value(),
        front.value(),
        profiles,
        flat_multi_normal,
        options);
    if (!flat_combined.hasValue())
        return 30;
    if (flat_combined.value().multi_normal_transition.applied)
        return 31;
    if (flat_combined.value().reserved_transition_cell_count == 0)
        return 32;
    if (flat_combined.value().regular_cell_count == 0)
        return 33;

    SurfaceMesh corner = mesh;
    corner.face_tags.assign(
        corner.faces.size(),
        {SurfaceBoundaryKind::Farfield, 0});
    corner.face_tags[0] = {SurfaceBoundaryKind::Wall, 1};
    corner.face_tags[2] = {SurfaceBoundaryKind::Wall, 1};
    corner.face_tags[5] = {SurfaceBoundaryKind::Wall, 1};
    const auto corner_topology =
        SurfaceTopologyBuilder{}.build(corner);
    assert(corner_topology.hasValue());
    const auto corner_patch = GrowthPatchBuilder{}.build(
        corner, corner_topology.value());
    assert(corner_patch.hasValue());
    const auto corner_front = GrowthFrontBuilder{}.buildInitial(
        corner, corner_patch.value());
    assert(corner_front.hasValue());
    std::vector<SourceVertexGrowthProfile> corner_profiles;
    for (const PatchVertex &vertex : corner_patch.value().vertices())
        corner_profiles.push_back(
            {vertex.source_vertex_id, {0.05, 1.0, 1}});

    MultiNormalOptions multi_normal;
    multi_normal.enabled = true;
    multi_normal.transition_height = 0.05;
    multi_normal.split_skewness_threshold = 0.5;
    const auto combined = generateReservedLayerTransition(
        corner,
        corner_topology.value(),
        corner_patch.value(),
        corner_front.value(),
        corner_profiles,
        multi_normal,
        options);
    assert(combined.hasValue());
    assert(combined.value().multi_normal_transition.applied);
    assert(!combined.value().multi_normal_transition
                .transition_cells.cells.empty());
    for (const CellMetadata &metadata :
         combined.value().multi_normal_transition.transition_cells.metadata)
        assert(metadata.role == CellRole::MultiNormalTransition);
    assert(combined.value().reserved_transition_cell_count == 0);
    assert(combined.value().mesh.cells.size() ==
           combined.value().multi_normal_transition
                   .transition_cells.cells.size() +
               combined.value().reserved_transition_cell_count +
               combined.value().regular_cell_count);
}
