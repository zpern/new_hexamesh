#include <algorithm>
#include <cassert>
#include <variant>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp>

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

    RegularLayerGrowthResult staircaseRegular(
        SurfaceMesh &surface, GrowthFront &front)
    {
        constexpr std::array<std::uint32_t,4> layers{1,2,3,4};
        for (std::uint32_t x = 0; x <= 4; ++x)
        {
            surface.vertices.push_back({Scalar(x),0,0});
            surface.vertices.push_back({Scalar(x),1,0});
        }
        for (std::uint32_t x = 0; x < 4; ++x)
            surface.faces.push_back(Quad{{
                VertexId(2*x), VertexId(2*x+2),
                VertexId(2*x+3), VertexId(2*x+1)}});
        surface.face_tags.resize(
            4, {SurfaceBoundaryKind::Wall, 12});
        front.layer = 0;
        front.faces = surface.faces;
        front.source_face_ids = {0,1,2,3};
        for (VertexId id = 0; id < surface.vertices.size(); ++id)
            front.vertices.push_back({surface.vertices[id], id});

        RegularLayerGrowthResult regular;
        regular.mesh.vertices = surface.vertices;
        std::array<std::vector<VertexId>,10> columns;
        for (VertexId id = 0; id < 10; ++id)
            columns[id].push_back(id);
        for (std::uint32_t x = 0; x <= 4; ++x)
        {
            const std::uint32_t height = x == 0 ? layers[0]
                : x == 4 ? layers[3]
                : std::max(layers[x-1], layers[x]);
            for (std::uint32_t layer = 1; layer <= height; ++layer)
                for (std::uint32_t y = 0; y < 2; ++y)
                {
                    const VertexId source = VertexId(2*x+y);
                    columns[source].push_back(VertexId(
                        regular.mesh.vertices.size()));
                    regular.mesh.vertices.push_back(
                        {Scalar(x),Scalar(y),Scalar(layer)});
                }
        }
        for (SurfaceFaceId face = 0; face < 4; ++face)
        {
            for (std::uint32_t layer = 1;
                 layer <= layers[face]; ++layer)
            {
                const VertexId a = VertexId(2*face);
                const VertexId b = VertexId(2*face+2);
                const VertexId c = VertexId(2*face+3);
                const VertexId d = VertexId(2*face+1);
                regular.mesh.cells.push_back(Hexa{{
                    columns[a][layer-1], columns[b][layer-1],
                    columns[c][layer-1], columns[d][layer-1],
                    columns[a][layer], columns[b][layer],
                    columns[c][layer], columns[d][layer]}});
                regular.mesh.metadata.push_back(
                    {CellRole::RegularLayer, face, layer});
            }
            regular.faces.push_back({
                face, layers[face], FaceGrowthStatus::Completed,
                FaceStopReason::VertexLayerLimit, layers[face]+1});
        }
        for (VertexId id = 0; id < 10; ++id)
            regular.layer_vertices.push_back({id,columns[id],0});
        return regular;
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

    SurfaceMesh triangle_surface;
    triangle_surface.vertices = {
        {0,0,0}, {1,0,0}, {0,1,0}};
    triangle_surface.faces = {Triangle{{0,1,2}}};
    triangle_surface.face_tags = {
        {SurfaceBoundaryKind::Wall, 9}};
    GrowthFront triangle_front;
    triangle_front.layer = 0;
    triangle_front.vertices = {
        {{0,0,0},0}, {{1,0,0},1}, {{0,1,0},2}};
    triangle_front.faces = triangle_surface.faces;
    triangle_front.source_face_ids = {0};

    RegularLayerGrowthResult triangle_regular;
    triangle_regular.mesh.vertices = {
        {0,0,0}, {1,0,0}, {0,1,0},
        {0,0,1}, {1,0,1}, {0,1,1},
        {0,0,2}, {1,0,2}, {0,1,2}};
    triangle_regular.mesh.cells = {
        Prism{{0,1,2,3,4,5}}, Prism{{3,4,5,6,7,8}}};
    triangle_regular.mesh.metadata = {
        {CellRole::RegularLayer, 0, 1},
        {CellRole::RegularLayer, 0, 2}};
    triangle_regular.faces = {
        {0,2,FaceGrowthStatus::Completed,
         FaceStopReason::VertexLayerLimit,3}};

    const auto triangle = finalizeIncrementalLayerTopology(
        triangle_surface, triangle_front, std::move(triangle_regular));
    assert(triangle.hasValue());
    assert(triangle.value().top_surface.faces.size() == 1);
    const std::array<VertexId,3> expected_triangle_top{6,7,8};
    assert(std::get<Triangle>(triangle.value().top_surface.faces.front())
               .vertex_ids == expected_triangle_top);

    RegularLayerGrowthResult zero_triangle_regular;
    zero_triangle_regular.mesh.vertices = triangle_surface.vertices;
    zero_triangle_regular.mesh.vertices.insert(
        zero_triangle_regular.mesh.vertices.end(),
        {{10,0,0},{11,0,0},{10,1,0},{10,0,1}});
    zero_triangle_regular.mesh.cells = {
        Tetra{{3,4,5,6}}};
    zero_triangle_regular.mesh.metadata = {
        {CellRole::RegularLayer,99,1}};
    zero_triangle_regular.faces = {
        {0,0,FaceGrowthStatus::Stopped,
         FaceStopReason::Collision,1}};
    zero_triangle_regular.layer_vertices = {
        {0,{0},0}, {1,{1},0}, {2,{2},0}};
    zero_triangle_regular.farfield_boundary.vertices =
        triangle_surface.vertices;
    zero_triangle_regular.farfield_boundary.faces =
        triangle_surface.faces;
    zero_triangle_regular.farfield_boundary.face_tags = {{
        SurfaceBoundaryKind::BoundaryLayerInterface,9}};
    const auto zero_triangle = finalizeIncrementalLayerTopology(
        triangle_surface, triangle_front,
        std::move(zero_triangle_regular));
    assert(zero_triangle.hasValue());
    assert(zero_triangle.value().mesh.cells.size() == 1);
    assert(zero_triangle.value().top_surface.faces.size() == 1);
    assert(std::holds_alternative<Triangle>(
        zero_triangle.value().top_surface.faces.front()));
    assert(zero_triangle.value().farfield_boundary.faces.size() == 1);
    assert(std::holds_alternative<Triangle>(
        zero_triangle.value().farfield_boundary.faces.front()));

    SurfaceMesh staircase_surface;
    GrowthFront staircase_front;
    auto staircase_regular = staircaseRegular(
        staircase_surface, staircase_front);
    const auto staircase = finalizeIncrementalLayerTopology(
        staircase_surface, staircase_front,
        std::move(staircase_regular));
    assert(staircase.hasValue());
    const std::array<std::uint32_t,4> expected_layers{1,2,3,4};
    for (std::size_t index = 1; index < expected_layers.size(); ++index)
        assert(expected_layers[index] - expected_layers[index-1] == 1);
    assert(expected_layers.back() - expected_layers.front() == 3);
    assert(std::all_of(
        staircase.value().top_surface.faces.begin(),
        staircase.value().top_surface.faces.end(),
        [](const SurfaceFace &face)
        { return std::holds_alternative<Triangle>(face); }));
    assert(count(staircase.value().mesh, CellType::Pyramid) > 20);
    assert(count(staircase.value().mesh, CellType::Tetra) > 8);

    SurfaceMesh rollback_surface;
    rollback_surface.vertices = {
        {0,0,0}, {0,1,0}, {1,0,0}, {1,1,0}, {2,0,0}, {2,1,0},
        {0,0,1}, {0,1,1}, {1,0,1}, {1,1,1}, {2,0,1}, {2,1,1},
        {0.2,0.2,0.05}, {0.8,0.2,0.05}, {0.5,0.8,0.05},
        {0.5,0.5,0.08}};
    rollback_surface.faces = {
        Quad{{0,2,3,1}}, Quad{{2,4,5,3}},
        Quad{{6,7,9,8}}, Quad{{8,9,11,10}},
        Quad{{0,6,8,2}}, Quad{{2,8,10,4}},
        Quad{{1,3,9,7}}, Quad{{3,5,11,9}},
        Quad{{0,1,7,6}}, Quad{{4,10,11,5}},
        Triangle{{12,14,13}}, Triangle{{12,13,15}},
        Triangle{{13,14,15}}, Triangle{{14,12,15}}};
    rollback_surface.face_tags = {
        {SurfaceBoundaryKind::Wall,20},
        {SurfaceBoundaryKind::Wall,20},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,30},
        {SurfaceBoundaryKind::Farfield,31},
        {SurfaceBoundaryKind::Farfield,31},
        {SurfaceBoundaryKind::Farfield,31},
        {SurfaceBoundaryKind::Farfield,31}};
    const auto rollback_topology =
        SurfaceTopologyBuilder{}.build(rollback_surface);
    assert(rollback_topology.hasValue());
    const auto rollback_patch = GrowthPatchBuilder{}.build(
        rollback_surface, rollback_topology.value());
    assert(rollback_patch.hasValue());
    const auto rollback_front = GrowthFrontBuilder{}.buildInitial(
        rollback_surface, rollback_patch.value());
    assert(rollback_front.hasValue());
    std::vector<SourceVertexGrowthProfile> rollback_profiles;
    for (const auto &vertex : rollback_patch.value().vertices())
        rollback_profiles.push_back({
            vertex.source_vertex_id, {0.1,1.0,1}});
    RegularLayerGrowthOptions rollback_options;
    rollback_options.isotropic_height = 100;
    rollback_options.cell_quality.maximum_skewness = 1;
    const auto rollback = generateIncrementalBoundaryLayers(
        rollback_surface, rollback_topology.value(),
        rollback_patch.value(), rollback_front.value(),
        rollback_profiles, rollback_options);
    assert(rollback.hasValue());
    assert(rollback.value().faces.size() == 2);
    assert(rollback.value().faces[0].accepted_layer_count == 0);
    assert(rollback.value().faces[1].accepted_layer_count == 0);
    assert(rollback.value().mesh.cells.empty());

    SurfaceMesh triangle_step_surface;
    triangle_step_surface.vertices = {
        {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}};
    triangle_step_surface.faces = {
        Triangle{{0,1,2}}, Triangle{{1,3,2}}};
    triangle_step_surface.face_tags.resize(
        2, {SurfaceBoundaryKind::Wall,40});
    GrowthFront triangle_step_front;
    triangle_step_front.layer = 0;
    triangle_step_front.faces = triangle_step_surface.faces;
    triangle_step_front.source_face_ids = {0,1};
    for (VertexId id = 0; id < 4; ++id)
        triangle_step_front.vertices.push_back(
            {triangle_step_surface.vertices[id],id});
    RegularLayerGrowthResult triangle_step_regular;
    triangle_step_regular.mesh.vertices = {
        {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0},
        {1,0,1}, {1,1,1}, {0,1,1}};
    triangle_step_regular.mesh.cells = {
        Prism{{1,3,2,4,5,6}}};
    triangle_step_regular.mesh.metadata = {
        {CellRole::RegularLayer,1,1}};
    triangle_step_regular.faces = {
        {0,0,FaceGrowthStatus::Stopped,
         FaceStopReason::Collision,1},
        {1,1,FaceGrowthStatus::Completed,
         FaceStopReason::VertexLayerLimit,2}};
    triangle_step_regular.layer_vertices = {
        {0,{0},0}, {1,{1,4},0},
        {2,{2,6},0}, {3,{3,5},0}};
    const auto triangle_step = finalizeIncrementalLayerTopology(
        triangle_step_surface, triangle_step_front,
        std::move(triangle_step_regular));
    assert(triangle_step.hasValue());
    assert(count(triangle_step.value().mesh, CellType::Prism) == 1);
    assert(count(triangle_step.value().mesh, CellType::Pyramid) == 1);
    assert(triangle_step.value().top_surface.faces.size() == 4);
    assert(std::all_of(
        triangle_step.value().top_surface.faces.begin(),
        triangle_step.value().top_surface.faces.end(),
        [](const SurfaceFace &face)
        { return std::holds_alternative<Triangle>(face); }));
}
