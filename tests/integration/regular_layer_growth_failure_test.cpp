#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makePrismMesh()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, 1, 0},
            Point3{0, 0, 1}, Point3{1, 0, 1}, Point3{0, 1, 1}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{3}, VertexId{4}, VertexId{5}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{4}, VertexId{3}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{2}, VertexId{0}, VertexId{3}, VertexId{5}}}};
        mesh.face_tags = {
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Farfield, 21},
            {SurfaceBoundaryKind::Farfield, 22},
            {SurfaceBoundaryKind::Farfield, 23}};
        return mesh;
    }

    bool sameFront(const GrowthFront &first, const GrowthFront &second)
    {
        if (first.layer != second.layer ||
            first.vertices.size() != second.vertices.size() ||
            first.faces.size() != second.faces.size() ||
            first.source_face_ids != second.source_face_ids)
        {
            return false;
        }
        for (std::size_t index = 0; index < first.vertices.size(); ++index)
        {
            const GrowthFrontVertex &left = first.vertices[index];
            const GrowthFrontVertex &right = second.vertices[index];
            if ((left.position - right.position).norm() != 0.0 ||
                (left.root_position - right.root_position).norm() != 0.0 ||
                left.source_vertex_id != right.source_vertex_id ||
                left.boundary.symmetry_region_ids !=
                    right.boundary.symmetry_region_ids ||
                (left.direction - right.direction).norm() != 0.0 ||
                left.actual_height != right.actual_height ||
                left.visibility_cosine != right.visibility_cosine ||
                left.complex_corner != right.complex_corner)
            {
                return false;
            }
        }
        return true;
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceMesh mesh = makePrismMesh();
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue()) return 2;
    const auto front_result = GrowthFrontBuilder{}.buildInitial(
        mesh, patch.value());
    if (!front_result.hasValue()) return 3;
    const GrowthFront front = front_result.value();
    const std::vector<SourceVertexGrowthProfile> profiles{
        {VertexId{3}, {0.1, 1.0, 1}},
        {VertexId{4}, {0.1, 1.0, 1}},
        {VertexId{5}, {0.1, 1.0, 1}}};

    GrowthFront malformed = front;
    malformed.vertices.back().source_vertex_id = VertexId{99};
    const auto mapping_failure = generateRegularLayers(
        mesh, topology.value(), patch.value(), malformed, profiles);
    const auto *mapping_error = mapping_failure.hasValue()
        ? nullptr
        : std::get_if<InvalidLayerFrontMapping>(&mapping_failure.error());
    if (mapping_error == nullptr || mapping_error->layer != 0) return 4;

    auto invalid_profiles = profiles;
    invalid_profiles[0].profile.first_height = 0.0;
    const auto profile_failure = generateRegularLayers(
        mesh, topology.value(), patch.value(), front, invalid_profiles);
    const auto *profile_wrapper = profile_failure.hasValue()
        ? nullptr
        : std::get_if<GrowthProfileFailure>(&profile_failure.error());
    const auto *height_error = profile_wrapper == nullptr
        ? nullptr
        : std::get_if<InvalidFirstHeight>(&profile_wrapper->cause);
    if (height_error == nullptr ||
        height_error->source_vertex_id != VertexId{3})
    {
        return 5;
    }

    GrowthFront degenerate = front;
    const Triangle &face = std::get<Triangle>(degenerate.faces[0]);
    degenerate.vertices[face.vertex_ids[1]].position =
        degenerate.vertices[face.vertex_ids[0]].position;
    const GrowthFront degenerate_before = degenerate;
    const auto front_failure = generateRegularLayers(
        mesh, topology.value(), patch.value(), degenerate, profiles);
    const auto *front_wrapper = front_failure.hasValue()
        ? nullptr
        : std::get_if<FrontEvaluationFailure>(&front_failure.error());
    const auto *face_error = front_wrapper == nullptr
        ? nullptr
        : std::get_if<DegenerateFrontFace>(&front_wrapper->cause);
    if (face_error == nullptr ||
        front_wrapper->target_layer != 1 ||
        face_error->source_face_id != SurfaceFaceId{1} ||
        !sameFront(degenerate, degenerate_before))
    {
        return 6;
    }

    RegularLayerGrowthOptions invalid_options;
    invalid_options.cell_quality.maximum_skewness = 2.0;
    const GrowthFront front_before = front;
    const auto quality_failure = generateRegularLayers(
        mesh,
        topology.value(),
        patch.value(),
        front,
        profiles,
        invalid_options);
    const auto *quality_wrapper = quality_failure.hasValue()
        ? nullptr
        : std::get_if<CellEvaluationFailure>(&quality_failure.error());
    if (quality_wrapper == nullptr ||
        quality_wrapper->layer != 1 ||
        quality_wrapper->source_face_id != SurfaceFaceId{1} ||
        quality_wrapper->cause.category !=
            VolumeCellEvaluationErrorCategory::InvalidMaximumSkewness ||
        !sameFront(front, front_before))
    {
        return 7;
    }

    return 0;
}
