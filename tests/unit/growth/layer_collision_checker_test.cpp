#include <cassert>
#include <cstddef>
#include <utility>

#include <boundary_mesh/growth/layer_collision_checker.hpp>

using namespace boundary_mesh;

namespace
{
    GrowthFront triangleFront(
        Scalar z,
        std::uint32_t layer,
        SurfaceFaceId source_face_id,
        VertexId source_offset)
    {
        GrowthFront front;
        front.layer = layer;
        front.vertices = {{0, 0, z}, {1, 0, z}, {0, 1, z}};
        front.faces = {Triangle{{0, 1, 2}}};
        front.source_vertex_ids = {
            source_offset,
            static_cast<VertexId>(source_offset + 1),
            static_cast<VertexId>(source_offset + 2)};
        front.source_face_ids = {source_face_id};
        front.vertex_boundaries.resize(3);
        return front;
    }

    LayerStepResult triangleStep(
        SurfaceFaceId source_face_id,
        VertexId source_offset)
    {
        LayerStepResult step;
        step.layer = 1;
        step.next_front = triangleFront(1.0, 1, source_face_id, source_offset);
        step.previous_front_vertex_indices = {0, 1, 2};
        step.previous_front_face_indices = {0};
        return step;
    }

    LayerStepResult liftedStep(const GrowthFront &current, Scalar height)
    {
        LayerStepResult step;
        step.layer = current.layer + 1;
        step.next_front = current;
        step.next_front.layer = step.layer;
        for (Point3 &point : step.next_front.vertices)
        {
            point.z() += height;
        }
        for (std::size_t index = 0;
             index < current.vertices.size();
             ++index)
        {
            step.previous_front_vertex_indices.push_back(index);
        }
        for (std::size_t index = 0;
             index < current.faces.size();
             ++index)
        {
            step.previous_front_face_indices.push_back(index);
        }
        return step;
    }

    void assertAdjacentCandidatesRemainActive(const GrowthFront &front)
    {
        const auto filtered = LayerCollisionChecker{}.filterSelfCollisions(
            front,
            liftedStep(front, Scalar{0.1}));
        assert(filtered.hasValue());
        assert(filtered.value().next_front.faces.size() ==
               front.faces.size());
        assert(filtered.value().stopped_faces.empty());
    }
}

int main()
{
    const GrowthFront current = triangleFront(0.0, 0, 4, 0);
    const LayerStepResult quality = triangleStep(4, 0);
    const CollisionTriangle obstacle{
        {{{0.5, -0.2, 0.2},
          {0.5, 0.8, 0.2},
          {0.5, 0.2, 1.2}}},
        {{{20, 0}, {21, 0}, {22, 0}}},
        CollisionOwnerKind::OriginalSurface,
        9};
    const auto obstacle_index = CollisionIndex::build({obstacle});
    assert(obstacle_index.hasValue());

    const ExposedBoundaryTracker empty_history;
    const auto filtered = LayerCollisionChecker{}.filterAgainstObstacles(
        obstacle_index.value(),
        empty_history,
        current,
        quality);
    assert(filtered.hasValue());
    assert(filtered.value().next_front.faces.empty());
    assert(filtered.value().stopped_faces.size() == 1);
    assert(filtered.value().stopped_faces.front().reason ==
           FaceStopReason::Collision);

    GrowthFront double_current = current;
    double_current.vertices.insert(
        double_current.vertices.end(),
        current.vertices.begin(),
        current.vertices.end());
    double_current.faces.push_back(Triangle{{3, 4, 5}});
    double_current.source_vertex_ids.insert(
        double_current.source_vertex_ids.end(), {10, 11, 12});
    double_current.vertex_boundaries.resize(6);
    double_current.source_face_ids.push_back(5);

    LayerStepResult double_step;
    double_step.layer = 1;
    double_step.next_front = double_current;
    double_step.next_front.layer = 1;
    for (Point3 &point : double_step.next_front.vertices)
    {
        point.z() = 1.0;
    }
    double_step.previous_front_vertex_indices = {0, 1, 2, 3, 4, 5};
    double_step.previous_front_face_indices = {0, 1};

    const auto self_filtered = LayerCollisionChecker{}.filterSelfCollisions(
        double_current,
        double_step);
    assert(self_filtered.hasValue());
    assert(self_filtered.value().next_front.faces.empty());
    assert(self_filtered.value().stopped_faces.size() == 2);

    GrowthFront adjacent;
    adjacent.layer = 0;
    adjacent.vertices = {
        {0, 0, 1}, {1, 0, 1}, {2, 0, 1},
        {0, 1, 1}, {1, 1, 1}, {2, 1, 1}};
    adjacent.faces = {
        Quad{{0, 1, 4, 3}},
        Quad{{1, 2, 5, 4}}};
    adjacent.source_vertex_ids = {6, 7, 8, 9, 10, 11};
    adjacent.source_face_ids = {2, 3};
    adjacent.vertex_boundaries.resize(6);

    LayerStepResult adjacent_step;
    adjacent_step.layer = 1;
    adjacent_step.next_front = adjacent;
    adjacent_step.next_front.layer = 1;
    for (Point3 &point : adjacent_step.next_front.vertices)
    {
        point.z() = 1.1;
    }
    adjacent_step.previous_front_vertex_indices = {0, 1, 2, 3, 4, 5};
    adjacent_step.previous_front_face_indices = {0, 1};
    const auto adjacent_filtered =
        LayerCollisionChecker{}.filterSelfCollisions(
            adjacent,
            adjacent_step);
    if (!adjacent_filtered.hasValue() ||
        adjacent_filtered.value().next_front.faces.size() != 2 ||
        !adjacent_filtered.value().stopped_faces.empty())
    {
        return 1;
    }

    GrowthFront vertex_adjacent;
    vertex_adjacent.vertices = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
        {0, 0, 0}, {-1, 0, 0}, {0, -1, 0}};
    vertex_adjacent.faces = {
        Triangle{{0, 1, 2}}, Triangle{{3, 4, 5}}};
    vertex_adjacent.source_vertex_ids = {20, 21, 22, 20, 23, 24};
    vertex_adjacent.source_face_ids = {20, 21};
    vertex_adjacent.vertex_boundaries.resize(6);
    assertAdjacentCandidatesRemainActive(vertex_adjacent);

    GrowthFront mixed_adjacent;
    mixed_adjacent.vertices = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
        {1, 0, 0}, {0, 0, 0}, {0, -1, 0}, {1, -1, 0}};
    mixed_adjacent.faces = {
        Triangle{{0, 1, 2}}, Quad{{3, 4, 5, 6}}};
    mixed_adjacent.source_vertex_ids = {
        30, 31, 32, 31, 30, 33, 34};
    mixed_adjacent.source_face_ids = {30, 31};
    mixed_adjacent.vertex_boundaries.resize(7);
    assertAdjacentCandidatesRemainActive(mixed_adjacent);

    GrowthFront reordered = double_current;
    std::swap(reordered.faces[0], reordered.faces[1]);
    std::swap(
        reordered.source_face_ids[0],
        reordered.source_face_ids[1]);
    const auto reordered_filtered =
        LayerCollisionChecker{}.filterSelfCollisions(
            reordered,
            liftedStep(reordered, Scalar{1.0}));
    assert(reordered_filtered.hasValue());
    assert(reordered_filtered.value().next_front.faces.empty());
    assert(reordered_filtered.value().stopped_faces.size() == 2);
    assert(reordered_filtered.value().stopped_faces[0].source_face_id == 4);
    assert(reordered_filtered.value().stopped_faces[1].source_face_id == 5);
}
