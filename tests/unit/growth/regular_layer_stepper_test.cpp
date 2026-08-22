#include <array>
#include <cmath>
#include <limits>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/regular_layer_stepper.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makePrismWithTopWall()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0},
            Point3{0.0, 1.0, 1.0}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{3}, VertexId{4}, VertexId{5}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{4}, VertexId{3}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{2}, VertexId{0}, VertexId{3}, VertexId{5}}}};
        mesh.face_tags = {
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Symmetry, 30},
            {SurfaceBoundaryKind::Symmetry, 31},
            {SurfaceBoundaryKind::Symmetry, 32}};
        return mesh;
    }

    SurfaceMesh makeHexaWithTopWall()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0},
            Point3{1.0, 1.0, 0.0}, Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0}, Point3{1.0, 0.0, 1.0},
            Point3{1.0, 1.0, 1.0}, Point3{0.0, 1.0, 1.0}};
        mesh.faces = {
            Quad{{VertexId{0}, VertexId{3}, VertexId{2}, VertexId{1}}},
            Quad{{VertexId{4}, VertexId{5}, VertexId{6}, VertexId{7}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{6}, VertexId{5}}},
            Quad{{VertexId{2}, VertexId{3}, VertexId{7}, VertexId{6}}},
            Quad{{VertexId{3}, VertexId{0}, VertexId{4}, VertexId{7}}}};
        mesh.face_tags = {
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Farfield, 21},
            {SurfaceBoundaryKind::Farfield, 22},
            {SurfaceBoundaryKind::Farfield, 23},
            {SurfaceBoundaryKind::Farfield, 24}};
        return mesh;
    }

    SurfaceMesh makeTetraWithTwoWallFaces()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{0}, VertexId{1}, VertexId{3}}},
            Triangle{{VertexId{1}, VertexId{2}, VertexId{3}}},
            Triangle{{VertexId{2}, VertexId{0}, VertexId{3}}}};
        mesh.face_tags = {
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Wall, 11},
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Farfield, 21}};
        return mesh;
    }

    SurfaceMesh makeDisconnectedMixedWallMesh()
    {
        SurfaceMesh mesh = makePrismWithTopWall();
        SurfaceMesh hexa = makeHexaWithTopWall();
        const VertexId offset = static_cast<VertexId>(mesh.vertices.size());
        for (Point3 point : hexa.vertices)
        {
            point.x() += 3.0;
            mesh.vertices.push_back(point);
        }
        for (const SurfaceFace &face : hexa.faces)
        {
            mesh.faces.push_back(std::visit(
                [&](const auto &value) -> SurfaceFace
                {
                    using Face = std::decay_t<decltype(value)>;
                    Face shifted = value;
                    for (VertexId &vertex_id : shifted.vertex_ids)
                    {
                        vertex_id += offset;
                    }
                    return shifted;
                },
                face));
        }
        mesh.face_tags.insert(
            mesh.face_tags.end(),
            hexa.face_tags.begin(),
            hexa.face_tags.end());
        return mesh;
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceMesh mesh = makePrismWithTopWall();
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue()) return 2;
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    if (!front.hasValue()) return 3;

    const std::vector<SourceVertexGrowthProfile> input_profiles{
        {VertexId{3}, {0.25, 1.0, 2}},
        {VertexId{4}, {0.25, 1.0, 2}},
        {VertexId{5}, {0.25, 1.0, 2}}};
    const auto profiles = GrowthProfileBuilder{}.build(
        patch.value(), input_profiles);
    if (!profiles.hasValue()) return 4;

    const auto result = RegularLayerStepper{}.step(
        front.value(),
        profiles.value(),
        buildFaceLayerConstraints(
            patch.value(), front.value(), profiles.value()).value(),
        RegularLayerGrowthOptions{});
    if (!result.hasValue()) return 5;

    const LayerStepResult &layer = result.value();
    if (layer.layer != 1 || layer.next_front.layer != 1 ||
        layer.next_front.vertices.size() != 3 ||
        layer.next_front.faces.size() != 1 ||
        layer.previous_front_vertex_indices !=
            std::vector<std::size_t>{0, 1, 2} ||
        layer.previous_front_face_indices !=
            std::vector<std::size_t>{0} ||
        !layer.stopped_faces.empty() ||
        !layer.completed_faces.empty())
    {
        return 6;
    }

    for (std::size_t index = 0; index < 3; ++index)
    {
        const Point3 expected =
            front.value().vertices[index].position +
            Vector3{0.0, 0.0, 0.25};
        if ((layer.next_front.vertices[index].position - expected).norm() >
            1e-12)
        {
            return 7;
        }
        const GrowthFrontVertex &vertex =
            layer.next_front.vertices[index];
        if (std::abs(vertex.actual_height - Scalar{0.25}) > 1e-12 ||
            (vertex.direction - Vector3::UnitZ()).norm() > 1e-12 ||
            std::abs(vertex.visibility_cosine - Scalar{1}) > 1e-12 ||
            vertex.complex_corner)
        {
            return 40;
        }
    }

    const auto *bottom = std::get_if<Triangle>(&front.value().faces[0]);
    const auto *top = std::get_if<Triangle>(&layer.next_front.faces[0]);
    if (bottom == nullptr || top == nullptr) return 8;
    const PrismPoints points{
        front.value().vertices[bottom->vertex_ids[0]].position,
        front.value().vertices[bottom->vertex_ids[1]].position,
        front.value().vertices[bottom->vertex_ids[2]].position,
        layer.next_front.vertices[top->vertex_ids[0]].position,
        layer.next_front.vertices[top->vertex_ids[1]].position,
        layer.next_front.vertices[top->vertex_ids[2]].position};
    const auto quality = evaluatePrism(points);
    if (!quality.hasValue() || !quality.value().acceptable ||
        quality.value().validity != VolumeCellValidity::Valid)
    {
        return 9;
    }

    const SurfaceMesh hexa_mesh = makeHexaWithTopWall();
    const auto hexa_topology = SurfaceTopologyBuilder{}.build(hexa_mesh);
    if (!hexa_topology.hasValue()) return 10;
    const auto hexa_patch = GrowthPatchBuilder{}.build(
        hexa_mesh, hexa_topology.value());
    if (!hexa_patch.hasValue()) return 11;
    const auto hexa_front = GrowthFrontBuilder{}.buildInitial(
        hexa_mesh, hexa_patch.value());
    if (!hexa_front.hasValue()) return 12;
    const std::vector<SourceVertexGrowthProfile> hexa_input_profiles{
        {VertexId{4}, {0.5, 1.0, 1}},
        {VertexId{5}, {0.5, 1.0, 1}},
        {VertexId{6}, {0.5, 1.0, 1}},
        {VertexId{7}, {0.5, 1.0, 1}}};
    const auto hexa_profiles = GrowthProfileBuilder{}.build(
        hexa_patch.value(), hexa_input_profiles);
    if (!hexa_profiles.hasValue()) return 13;
    const auto hexa_step = RegularLayerStepper{}.step(
        hexa_front.value(),
        hexa_profiles.value(),
        buildFaceLayerConstraints(
            hexa_patch.value(),
            hexa_front.value(),
            hexa_profiles.value()).value());
    if (!hexa_step.hasValue() ||
        hexa_step.value().next_front.vertices.size() != 4 ||
        hexa_step.value().next_front.faces.size() != 1)
    {
        return 14;
    }
    const auto *hexa_bottom = std::get_if<Quad>(
        &hexa_front.value().faces[0]);
    const auto *hexa_top = std::get_if<Quad>(
        &hexa_step.value().next_front.faces[0]);
    if (hexa_bottom == nullptr || hexa_top == nullptr) return 15;
    HexaPoints hexa_points;
    for (std::size_t local = 0; local < 4; ++local)
    {
        hexa_points[local] = hexa_front.value().vertices[
            hexa_bottom->vertex_ids[local]].position;
        hexa_points[local + 4] = hexa_step.value().next_front.vertices[
            hexa_top->vertex_ids[local]].position;
    }
    const auto hexa_quality = evaluateHexa(hexa_points);
    if (!hexa_quality.hasValue() ||
        !hexa_quality.value().acceptable ||
        hexa_quality.value().validity != VolumeCellValidity::Valid)
    {
        return 16;
    }

    const SurfaceMesh mixed_mesh = makeDisconnectedMixedWallMesh();
    const auto mixed_topology = SurfaceTopologyBuilder{}.build(mixed_mesh);
    if (!mixed_topology.hasValue()) return 26;
    const auto mixed_patch = GrowthPatchBuilder{}.build(
        mixed_mesh, mixed_topology.value());
    if (!mixed_patch.hasValue()) return 27;
    const auto mixed_front = GrowthFrontBuilder{}.buildInitial(
        mixed_mesh, mixed_patch.value());
    if (!mixed_front.hasValue()) return 28;
    std::vector<SourceVertexGrowthProfile> mixed_profile_input;
    for (const PatchVertex &vertex : mixed_patch.value().vertices())
    {
        mixed_profile_input.push_back(
            {vertex.source_vertex_id, {0.2, 1.0, 1}});
    }
    const auto mixed_profiles = GrowthProfileBuilder{}.build(
        mixed_patch.value(), mixed_profile_input);
    if (!mixed_profiles.hasValue()) return 29;
    const auto mixed_step = RegularLayerStepper{}.step(
        mixed_front.value(),
        mixed_profiles.value(),
        buildFaceLayerConstraints(
            mixed_patch.value(),
            mixed_front.value(),
            mixed_profiles.value()).value());
    if (!mixed_step.hasValue() ||
        mixed_step.value().next_front.vertices.size() != 7 ||
        mixed_step.value().next_front.faces.size() != 2 ||
        std::get_if<Triangle>(&mixed_step.value().next_front.faces[0]) ==
            nullptr ||
        std::get_if<Quad>(&mixed_step.value().next_front.faces[1]) == nullptr)
    {
        return 30;
    }

    const std::vector<SourceVertexGrowthProfile> ratio_profile_input{
        {VertexId{3}, {0.25, 2.0, 2}},
        {VertexId{4}, {0.25, 2.0, 2}},
        {VertexId{5}, {0.25, 2.0, 2}}};
    const auto ratio_profiles = GrowthProfileBuilder{}.build(
        patch.value(), ratio_profile_input);
    if (!ratio_profiles.hasValue()) return 31;
    GrowthFront layer1 = front.value();
    layer1.layer = 1;
    for (GrowthFrontVertex &vertex : layer1.vertices)
    {
        vertex.actual_height = Scalar{0.08};
        vertex.direction = Vector3::UnitZ();
    }
    const auto ratio_step = RegularLayerStepper{}.step(
        layer1,
        ratio_profiles.value(),
        buildFaceLayerConstraints(
            patch.value(), front.value(), ratio_profiles.value()).value());
    if (!ratio_step.hasValue()) return 32;
    for (std::size_t index = 0; index < 3; ++index)
    {
        const Point3 expected =
            layer1.vertices[index].position + Vector3{0.0, 0.0, 0.16};
        if ((ratio_step.value().next_front.vertices[index].position -
             expected).norm() > 1e-12)
        {
            return 33;
        }
    }

    const std::vector<SourceVertexGrowthProfile> overflow_profile_input{
        {VertexId{3}, {std::numeric_limits<Scalar>::max(), 2.0, 2}},
        {VertexId{4}, {0.25, 1.0, 2}},
        {VertexId{5}, {0.25, 1.0, 2}}};
    const auto overflow_profiles = GrowthProfileBuilder{}.build(
        patch.value(), overflow_profile_input);
    if (!overflow_profiles.hasValue()) return 34;
    GrowthFront overflow_layer = layer1;
    overflow_layer.vertices[0].actual_height =
        std::numeric_limits<Scalar>::max();
    const auto overflow_step = RegularLayerStepper{}.step(
        overflow_layer,
        overflow_profiles.value(),
        buildFaceLayerConstraints(
            patch.value(), front.value(), overflow_profiles.value()).value());
    const auto *overflow_error = overflow_step.hasValue()
        ? nullptr
        : std::get_if<NonFiniteLayerHeight>(&overflow_step.error());
    if (overflow_error == nullptr ||
        overflow_error->source_vertex_id != VertexId{3} ||
        overflow_error->layer != 2)
    {
        return 35;
    }

    const std::vector<SourceVertexGrowthProfile> limited_profiles_input{
        {VertexId{3}, {0.25, 1.0, 4}},
        {VertexId{4}, {0.25, 1.0, 5}},
        {VertexId{5}, {0.25, 1.0, 6}}};
    const auto limited_profiles = GrowthProfileBuilder{}.build(
        patch.value(), limited_profiles_input);
    if (!limited_profiles.hasValue()) return 17;
    GrowthFront layer4 = front.value();
    layer4.layer = 4;
    const auto completed = RegularLayerStepper{}.step(
        layer4,
        limited_profiles.value(),
        buildFaceLayerConstraints(
            patch.value(), front.value(), limited_profiles.value()).value());
    if (!completed.hasValue() ||
        !completed.value().next_front.faces.empty() ||
        completed.value().completed_faces.size() != 1 ||
        completed.value().completed_faces[0].reason !=
            FaceStopReason::VertexLayerLimit ||
        completed.value().completed_faces[0].layer != 5)
    {
        return 18;
    }

    auto constrained_faces = buildFaceLayerConstraints(
        patch.value(), front.value(), limited_profiles.value()).value();
    FaceLayerConstraint *constrained_face = constrained_faces.find(
        front.value().source_face_ids[0]);
    if (constrained_face == nullptr) return 36;
    constrained_face->allowed_layer_count = 4;
    constrained_face->limit_kind = FaceLayerLimitKind::NeighborConstraint;
    const auto neighbor_stopped = RegularLayerStepper{}.step(
        layer4,
        limited_profiles.value(),
        constrained_faces);
    if (!neighbor_stopped.hasValue() ||
        !neighbor_stopped.value().next_front.faces.empty() ||
        neighbor_stopped.value().stopped_faces.size() != 1 ||
        neighbor_stopped.value().stopped_faces[0].reason !=
            FaceStopReason::NeighborLayerConstraint)
    {
        return 37;
    }

    GrowthFront degenerate_layer4 = layer4;
    degenerate_layer4.vertices[1].position =
        degenerate_layer4.vertices[0].position;
    const auto prefiltered = RegularLayerStepper{}.step(
        degenerate_layer4,
        limited_profiles.value(),
        buildFaceLayerConstraints(
            patch.value(), front.value(), limited_profiles.value()).value());
    if (!prefiltered.hasValue() ||
        prefiltered.value().completed_faces.size() != 1)
    {
        return 38;
    }

    constrained_face->limit_kind = FaceLayerLimitKind::DirectStop;
    constrained_face->direct_reason = FaceStopReason::Collision;
    const auto invalid_direct = RegularLayerStepper{}.step(
        layer4,
        limited_profiles.value(),
        constrained_faces);
    const auto *constraint_error = invalid_direct.hasValue()
        ? nullptr
        : std::get_if<InvalidFaceConstraintState>(&invalid_direct.error());
    if (constraint_error == nullptr ||
        constraint_error->source_face_id != front.value().source_face_ids[0] ||
        constraint_error->layer != 5)
    {
        return 39;
    }

    const SurfaceMesh tetra_mesh = makeTetraWithTwoWallFaces();
    const auto tetra_topology = SurfaceTopologyBuilder{}.build(tetra_mesh);
    if (!tetra_topology.hasValue()) return 19;
    const auto tetra_patch = GrowthPatchBuilder{}.build(
        tetra_mesh, tetra_topology.value());
    if (!tetra_patch.hasValue()) return 20;
    const auto tetra_front = GrowthFrontBuilder{}.buildInitial(
        tetra_mesh, tetra_patch.value());
    if (!tetra_front.hasValue()) return 21;
    const std::vector<SourceVertexGrowthProfile> tetra_profile_input{
        {VertexId{0}, {0.1, 1.0, 1}},
        {VertexId{1}, {0.1, 1.0, 1}},
        {VertexId{2}, {0.1, 1.0, 0}},
        {VertexId{3}, {0.1, 1.0, 1}}};
    const auto tetra_profiles = GrowthProfileBuilder{}.build(
        tetra_patch.value(), tetra_profile_input);
    if (!tetra_profiles.hasValue()) return 22;
    const auto tetra_step = RegularLayerStepper{}.step(
        tetra_front.value(),
        tetra_profiles.value(),
        buildFaceLayerConstraints(
            tetra_patch.value(),
            tetra_front.value(),
            tetra_profiles.value()).value());
    if (!tetra_step.hasValue() ||
        tetra_step.value().completed_faces.size() != 1 ||
        tetra_step.value().completed_faces[0].source_face_id !=
            SurfaceFaceId{0} ||
        tetra_step.value().next_front.faces.size() != 1 ||
        tetra_step.value().next_front.source_face_ids !=
            std::vector<SurfaceFaceId>{SurfaceFaceId{1}} ||
        tetra_step.value().next_front.vertices[0].source_vertex_id !=
            VertexId{0} ||
        tetra_step.value().next_front.vertices[1].source_vertex_id !=
            VertexId{1} ||
        tetra_step.value().next_front.vertices[2].source_vertex_id !=
            VertexId{3})
    {
        return 23;
    }

    const std::vector<SourceVertexGrowthProfile> skewed_profile_input{
        {VertexId{4}, {0.25, 1.0, 1}},
        {VertexId{5}, {0.50, 1.0, 1}},
        {VertexId{6}, {0.75, 1.0, 1}},
        {VertexId{7}, {1.00, 1.0, 1}}};
    const auto skewed_profiles = GrowthProfileBuilder{}.build(
        hexa_patch.value(), skewed_profile_input);
    if (!skewed_profiles.hasValue()) return 24;
    RegularLayerGrowthOptions strict_options;
    strict_options.cell_quality.maximum_skewness = 0.05;
    const auto stopped = RegularLayerStepper{}.step(
        hexa_front.value(),
        skewed_profiles.value(),
        buildFaceLayerConstraints(
            hexa_patch.value(),
            hexa_front.value(),
            skewed_profiles.value()).value(),
        strict_options);
    if (!stopped.hasValue() ||
        stopped.value().stopped_faces.size() != 1 ||
        stopped.value().stopped_faces[0].reason !=
            FaceStopReason::SkewnessExceeded ||
        !stopped.value().next_front.vertices.empty() ||
        !stopped.value().next_front.faces.empty() ||
        !stopped.value().previous_front_vertex_indices.empty() ||
        !stopped.value().previous_front_face_indices.empty())
    {
        return 25;
    }

    return 0;
}
