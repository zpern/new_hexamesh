#include <cassert>
#include <vector>

#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/growth/farfield_boundary_builder.hpp>

using namespace boundary_mesh;

int main()
{
    SurfaceMesh original;
    original.vertices = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
        {0, 0, 2}, {1, 0, 2}, {0, 1, 2}};
    original.faces = {
        Triangle{{0, 1, 2}},
        Triangle{{3, 4, 5}}};
    original.face_tags = {
        {SurfaceBoundaryKind::Wall, 9},
        {SurfaceBoundaryKind::Farfield, 4}};

    const BoundaryFace bottom{
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
        {{0, 0}, {1, 0}, {2, 0}},
        0,
        9};
    const BoundaryFace top{
        {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}},
        {{0, 1}, {1, 1}, {2, 1}},
        0,
        9};

    ExposedBoundaryTracker tracker;
    const auto update = tracker.prepare({{
        bottom,
        top,
        {{SurfaceBoundaryKind::Symmetry, 30},
         {SurfaceBoundaryKind::Internal, 40},
         {SurfaceBoundaryKind::BoundaryLayerInterface, 9}}}});
    assert(update.hasValue());
    tracker.apply(update.value());

    const auto result = buildFarfieldBoundary(original, tracker, {});
    assert(result.hasValue());
    assert(result.value().faces.size() == 5);
    assert(result.value().faces.size() == result.value().face_tags.size());
    assert(result.value().face_tags.front().kind ==
           SurfaceBoundaryKind::Farfield);
    std::size_t interface_count = 0;
    std::size_t symmetry_count = 0;
    std::size_t internal_count = 0;
    for (const SurfaceBoundaryTag tag : result.value().face_tags)
    {
        interface_count += tag.kind ==
            SurfaceBoundaryKind::BoundaryLayerInterface;
        symmetry_count += tag.kind == SurfaceBoundaryKind::Symmetry;
        internal_count += tag.kind == SurfaceBoundaryKind::Internal;
    }
    assert(interface_count == 2);
    assert(symmetry_count == 1);
    assert(internal_count == 1);

    const auto top_only = extractBoundaryLayerTop(result.value());
    assert(top_only.hasValue());
    assert(top_only.value().faces.size() == 2);
    for (const SurfaceBoundaryTag tag : top_only.value().face_tags)
        assert(tag.kind == SurfaceBoundaryKind::BoundaryLayerInterface);

    std::vector<bool> used(result.value().vertices.size(), false);
    bool reversed_top_found = false;
    for (const SurfaceFace &face : result.value().faces)
    {
        std::visit(
            [&](const auto &value)
            {
                for (const VertexId vertex_id : value.vertex_ids)
                {
                    used[static_cast<std::size_t>(vertex_id)] = true;
                }
                bool top_face = true;
                for (const VertexId vertex_id : value.vertex_ids)
                {
                    top_face = top_face &&
                        result.value().vertices[vertex_id].z() == 1.0;
                }
                if (top_face)
                {
                    const Vector3 edge0 =
                        result.value().vertices[value.vertex_ids[1]] -
                        result.value().vertices[value.vertex_ids[0]];
                    const Vector3 edge1 =
                        result.value().vertices[value.vertex_ids[2]] -
                        result.value().vertices[value.vertex_ids[0]];
                    reversed_top_found = edge0.cross(edge1).z() < 0.0;
                }
            },
            face);
    }
    assert(reversed_top_found);
    for (const bool referenced : used)
    {
        assert(referenced);
    }

    SurfaceMesh zero_layer_original;
    zero_layer_original.vertices = {
        {0, 0, 2}, {1, 0, 2}, {0, 1, 2},
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
        {2, 0, 0}, {3, 0, 0}, {3, 1, 0}, {2, 1, 0}};
    zero_layer_original.faces = {
        Triangle{{0, 1, 2}},
        Triangle{{3, 4, 5}},
        Quad{{6, 7, 8, 9}}};
    zero_layer_original.face_tags = {
        {SurfaceBoundaryKind::Farfield, 4},
        {SurfaceBoundaryKind::Wall, 9},
        {SurfaceBoundaryKind::Wall, 10}};

    const auto zero_layer = buildFarfieldBoundary(
        zero_layer_original,
        ExposedBoundaryTracker{},
        {SurfaceFaceId{1}, SurfaceFaceId{2}});
    assert(zero_layer.hasValue());
    assert(zero_layer.value().faces.size() == 3);
    assert(zero_layer.value().face_tags.size() == 3);
    assert(zero_layer.value().face_tags[1].kind ==
           SurfaceBoundaryKind::BoundaryLayerInterface);
    assert(zero_layer.value().face_tags[1].region_id == 9);
    assert(zero_layer.value().face_tags[2].kind ==
           SurfaceBoundaryKind::BoundaryLayerInterface);
    assert(zero_layer.value().face_tags[2].region_id == 10);

    const auto *fallback_triangle =
        std::get_if<Triangle>(&zero_layer.value().faces[1]);
    const auto *fallback_quad =
        std::get_if<Quad>(&zero_layer.value().faces[2]);
    assert(fallback_triangle != nullptr);
    assert(fallback_quad != nullptr);

    const auto normalZ = [&](const auto &face)
    {
        const Point3 &first = zero_layer.value().vertices[
            static_cast<std::size_t>(face.vertex_ids[0])];
        const Point3 &second = zero_layer.value().vertices[
            static_cast<std::size_t>(face.vertex_ids[1])];
        const Point3 &third = zero_layer.value().vertices[
            static_cast<std::size_t>(face.vertex_ids[2])];
        return (second - first).cross(third - first).z();
    };
    assert(normalZ(*fallback_triangle) < 0.0);
    assert(normalZ(*fallback_quad) < 0.0);

    const auto duplicate = buildFarfieldBoundary(
        zero_layer_original,
        ExposedBoundaryTracker{},
        {SurfaceFaceId{1}, SurfaceFaceId{1}});
    assert(!duplicate.hasValue());
    assert(duplicate.error() == SpatialError::InvalidTopologyReference);

    const auto out_of_range = buildFarfieldBoundary(
        zero_layer_original,
        ExposedBoundaryTracker{},
        {SurfaceFaceId{99}});
    assert(!out_of_range.hasValue());
    assert(out_of_range.error() == SpatialError::InvalidTopologyReference);

    const auto non_wall = buildFarfieldBoundary(
        zero_layer_original,
        ExposedBoundaryTracker{},
        {SurfaceFaceId{0}});
    assert(!non_wall.hasValue());
    assert(non_wall.error() == SpatialError::InvalidTopologyReference);
}
