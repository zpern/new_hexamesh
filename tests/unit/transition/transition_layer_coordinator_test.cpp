#include <algorithm>
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

    const auto two_high = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 2}, {2, 3}, {4, 3}});
    assert(two_high.hasValue());
    const auto &low = findFace(two_high.value(), 1);
    assert(low.high_edge_local_index.has_value());
    assert(findFace(two_high.value(), 2).layers.trial_layers == 3);
    assert(findFace(two_high.value(), 4).layers.trial_layers == 2);

    const auto reversed = coordinator.coordinate(
        patch.value(), topology.value(), {{4, 3}, {2, 3}, {1, 2}});
    assert(reversed.hasValue());
    assert(findFace(reversed.value(), 2).layers.trial_layers == 3);
    assert(findFace(reversed.value(), 4).layers.trial_layers == 2);

    const auto large_difference = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 1}, {2, 5}, {4, 1}});
    assert(large_difference.hasValue());
    assert(findFace(large_difference.value(), 2).layers.occupied_layers <=
           findFace(large_difference.value(), 1).layers.occupied_layers + 1);

    const auto missing = coordinator.coordinate(
        patch.value(), topology.value(), {{1, 1}, {2, 2}});
    assert(!missing.hasValue());
}
