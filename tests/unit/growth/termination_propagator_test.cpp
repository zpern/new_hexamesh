#include <cassert>

#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/termination_propagator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

using namespace boundary_mesh;

int main()
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

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    assert(front.hasValue());

    std::vector<SourceVertexGrowthProfile> source_profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        source_profiles.push_back({vertex.source_vertex_id, {0.1, 1.0, 10}});
    }
    const auto profiles = GrowthProfileBuilder{}.build(
        patch.value(), source_profiles);
    assert(profiles.hasValue());
    const auto initial = buildFaceLayerConstraints(
        patch.value(), front.value(), profiles.value());
    assert(initial.hasValue());

    const auto propagator = TerminationPropagator::build(
        patch.value(), topology.value());
    assert(propagator.hasValue());
    assert((propagator.value().neighbors(2) ==
            std::vector<SurfaceFaceId>{1}));
    assert((propagator.value().neighbors(1) ==
            std::vector<SurfaceFaceId>{2, 4}));

    auto diff1 = initial.value();
    diff1.find(2)->allowed_layer_count = 2;
    const auto changed1 = propagator.value().propagateInitial(diff1, 1);
    assert(changed1.hasValue());
    assert(diff1.find(2)->allowed_layer_count == 2);
    assert(diff1.find(1)->allowed_layer_count == 3);
    assert(diff1.find(4)->allowed_layer_count == 4);

    auto diff0 = initial.value();
    diff0.find(2)->allowed_layer_count = 2;
    const auto changed0 = propagator.value().propagateInitial(diff0, 0);
    assert(changed0.hasValue());
    assert(diff0.find(1)->allowed_layer_count == 2);
    assert(diff0.find(4)->allowed_layer_count == 2);
}
