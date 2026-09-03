#include <algorithm>
#include <cassert>
#include <variant>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/transition/incremental_boundary_layer_generator.hpp>

using namespace boundary_mesh;

namespace
{
    struct Fixture
    {
        SurfaceMesh mesh;
        SurfaceTopology topology;
        GrowthPatch patch;
        GrowthFront front;
    };

    Fixture fixture()
    {
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
        auto topology = SurfaceTopologyBuilder{}.build(mesh);
        assert(topology.hasValue());
        auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
        assert(patch.hasValue());
        auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
        assert(front.hasValue());
        return {std::move(mesh), std::move(topology.value()),
                std::move(patch.value()), std::move(front.value())};
    }

    std::size_t count(const VolumeMesh &mesh, CellType type)
    {
        return std::count_if(mesh.cells.begin(), mesh.cells.end(),
            [type](const VolumeCell &cell) { return cellType(cell) == type; });
    }

    RegularLayerGrowthResult generate(std::uint32_t layers)
    {
        auto data = fixture();
        std::vector<SourceVertexGrowthProfile> profiles;
        for (const auto &vertex : data.patch.vertices())
            profiles.push_back({vertex.source_vertex_id,
                                {0.05, 1.0, layers}});
        RegularLayerGrowthOptions options;
        options.isotropic_height = 100;
        const auto result = generateIncrementalBoundaryLayers(
            data.mesh, data.topology, data.patch, data.front,
            profiles, options);
        assert(result.hasValue());
        return result.value();
    }
}

int main()
{
    const auto one = generate(1);
    assert(count(one.mesh, CellType::Hexa) == 0);
    assert(count(one.mesh, CellType::Pyramid) == 5);
    assert(count(one.mesh, CellType::Tetra) == 2);
    assert(one.top_surface.faces.size() == 2);
    for (const auto &face : one.top_surface.faces)
        assert(std::holds_alternative<Triangle>(face));
    for (std::size_t index = 0;
         index < one.farfield_boundary.faces.size(); ++index)
        if (one.farfield_boundary.face_tags[index].kind ==
            SurfaceBoundaryKind::BoundaryLayerInterface)
            assert(std::holds_alternative<Triangle>(
                one.farfield_boundary.faces[index]));

    const auto two = generate(2);
    assert(count(two.mesh, CellType::Hexa) == 1);
    assert(count(two.mesh, CellType::Pyramid) == 5);
    assert(count(two.mesh, CellType::Tetra) == 2);
    assert(two.mesh.cells.size() == two.mesh.metadata.size());
    assert(std::all_of(two.top_surface.faces.begin(),
                      two.top_surface.faces.end(),
        [](const SurfaceFace &face)
        { return std::holds_alternative<Triangle>(face); }));

    SurfaceMesh strip_surface;
    strip_surface.vertices = {
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {2,0,0}, {2,1,0}};
    strip_surface.faces = {
        Quad{{0,1,2,3}}, Quad{{1,4,5,2}}};
    strip_surface.face_tags.resize(
        2, {SurfaceBoundaryKind::Wall, 7});
    GrowthFront strip_front;
    strip_front.layer = 0;
    for (VertexId id = 0; id < 6; ++id)
        strip_front.vertices.push_back(
            {strip_surface.vertices[id], id});
    strip_front.faces = strip_surface.faces;
    strip_front.source_face_ids = {0,1};

    RegularLayerGrowthResult strip_regular;
    strip_regular.mesh.vertices = strip_surface.vertices;
    strip_regular.mesh.vertices.insert(
        strip_regular.mesh.vertices.end(),
        {{1,0,1}, {2,0,1}, {2,1,1}, {1,1,1}});
    strip_regular.mesh.cells.push_back(
        Hexa{{1,4,5,2,6,7,8,9}});
    strip_regular.mesh.metadata.push_back(
        {CellRole::RegularLayer, 1, 1});
    strip_regular.faces = {
        {0,0,FaceGrowthStatus::Completed,
         FaceStopReason::VertexLayerLimit,1},
        {1,1,FaceGrowthStatus::Completed,
         FaceStopReason::VertexLayerLimit,2}};
    strip_regular.layer_vertices = {
        {0,{0},0}, {1,{1,6},0}, {2,{2,9},0},
        {3,{3},0}, {4,{4,7},0}, {5,{5,8},0}};

    const auto strip = finalizeIncrementalLayerTopology(
        strip_surface, strip_front, std::move(strip_regular));
    assert(strip.hasValue());
    assert(count(strip.value().mesh, CellType::Pyramid) == 6);
    assert(count(strip.value().mesh, CellType::Tetra) == 3);
    assert(count(strip.value().mesh, CellType::Hexa) == 0);
    assert(strip.value().mesh.cells.size() ==
           strip.value().mesh.metadata.size());
    assert(strip.value().top_surface.faces.size() == 6);
    assert(std::all_of(strip.value().top_surface.faces.begin(),
                      strip.value().top_surface.faces.end(),
        [](const SurfaceFace &face)
        { return std::holds_alternative<Triangle>(face); }));
}
