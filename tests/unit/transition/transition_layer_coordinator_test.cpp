#include <algorithm>

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <optional>
#include <utility>
#include <vector>

#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/transition/transition_layer_coordinator.hpp>

using namespace boundary_mesh;

namespace
{
    SurfaceMesh makeMesh()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
            {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
        mesh.faces = {
            Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
            Quad{{0, 1, 5, 4}}, Quad{{1, 2, 6, 5}},
            Quad{{2, 3, 7, 6}}, Quad{{3, 0, 4, 7}}};
        mesh.face_tags.resize(6, {SurfaceBoundaryKind::Farfield, 0});
        mesh.face_tags[1] = {SurfaceBoundaryKind::Wall, 1};
        mesh.face_tags[2] = {SurfaceBoundaryKind::Wall, 1};
        mesh.face_tags[4] = {SurfaceBoundaryKind::Wall, 1};
        return mesh;
    }

    SurfaceMesh makeAdjacentMesh()
    {
        SurfaceMesh mesh = makeMesh();
        mesh.face_tags[4] = {SurfaceBoundaryKind::Farfield, 0};
        mesh.face_tags[3] = {SurfaceBoundaryKind::Wall, 1};
        return mesh;
    }

    SurfaceMesh makeCornerFanMesh()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            {0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {0, 1, 0},
            {-1, 0, 0}, {0, -1, 0}};
        mesh.faces = {
            Triangle{{0, 2, 3}}, Triangle{{0, 3, 4}},
            Triangle{{0, 4, 5}}, Triangle{{0, 5, 2}},
            Triangle{{1, 3, 2}}, Triangle{{1, 4, 3}},
            Triangle{{1, 5, 4}}, Triangle{{1, 2, 5}}};
        mesh.face_tags.resize(
            mesh.faces.size(), {SurfaceBoundaryKind::Wall, 1});
        return mesh;
    }

    const CoordinatedTransitionFace &findFace(
        const std::vector<CoordinatedTransitionFace> &faces,
        SurfaceFaceId id)
    {
        const auto found = std::find_if(
            faces.begin(), faces.end(),
            [id](const CoordinatedTransitionFace &face)
            { return face.layers.source_face_id == id; });
        assert(found != faces.end());
        return *found;
    }
}

int main()
{
    const SurfaceMesh mesh = makeMesh();
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());

    const TransitionLayerCoordinator coordinator;

    const auto zero_one = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 0}, {2, 1}, {4, 0}});
    assert(zero_one.hasValue());
    assert(findFace(zero_one.value(), 1).layers.occupied_layers == 0);
    assert(!findFace(zero_one.value(), 1)
                .high_edge_local_index.has_value());

    const auto one_two = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 1}, {2, 2}, {4, 1}});
    assert(one_two.hasValue());
    assert(findFace(one_two.value(), 1).layers.occupied_layers == 0);
    assert(findFace(one_two.value(), 1).high_edge_local_index.has_value());

    const auto opposite_two_high = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 2}, {2, 3}, {4, 3}});
    assert(!opposite_two_high.hasValue());
    assert(std::holds_alternative<MultipleTransitionHighEdges>(
        opposite_two_high.error()));
    assert(std::get<MultipleTransitionHighEdges>(
               opposite_two_high.error()).high_edge_local_indices.size() == 2);

    const auto reversed = coordinator.coordinate(
        patch.value(), topology.value(), {{4, 3}, {2, 3}, {1, 2}});
    assert(!reversed.hasValue());

    const SurfaceMesh adjacent_mesh = makeAdjacentMesh();
    const auto adjacent_topology =
        SurfaceTopologyBuilder{}.build(adjacent_mesh);
    assert(adjacent_topology.hasValue());
    const auto adjacent_patch = GrowthPatchBuilder{}.build(
        adjacent_mesh, adjacent_topology.value());
    assert(adjacent_patch.hasValue());
    const auto adjacent_two_high = coordinator.coordinate(
        adjacent_patch.value(), adjacent_topology.value(),
        {{1, 2}, {2, 3}, {3, 3}});
    assert(!adjacent_two_high.hasValue());
    assert(std::holds_alternative<MultipleTransitionHighEdges>(
        adjacent_two_high.error()));

    const SurfaceMesh corner_fan = makeCornerFanMesh();
    const auto corner_topology =
        SurfaceTopologyBuilder{}.build(corner_fan);
    assert(corner_topology.hasValue());
    const auto corner_patch = GrowthPatchBuilder{}.build(
        corner_fan, corner_topology.value());
    assert(corner_patch.hasValue());
    const auto corner_violation = coordinator.coordinate(
        corner_patch.value(), corner_topology.value(),
        {{0, 2}, {1, 2}, {2, 3}, {3, 2},
         {4, 3}, {5, 2}, {6, 2}, {7, 2}});
    assert(!corner_violation.hasValue());
    assert(std::holds_alternative<TransitionCornerLayerViolation>(
        corner_violation.error()));

    const auto valid_corner_fan = coordinator.coordinate(
        corner_patch.value(), corner_topology.value(),
        {{0, 2}, {1, 2}, {2, 2}, {3, 2},
         {4, 3}, {5, 2}, {6, 2}, {7, 2}});
    assert(valid_corner_fan.hasValue());

    const auto large_difference = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 1}, {2, 5}, {4, 1}});
    assert(!large_difference.hasValue());
    assert(std::holds_alternative<UncoordinatedTransitionLayerDifference>(
        large_difference.error()));

    const auto missing = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 1}, {2, 2}});
    assert(!missing.hasValue());
}
