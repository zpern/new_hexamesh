#include <algorithm>
#include <cassert>
#include <vector>

#include <boundary_mesh/transition/corner_suppression.hpp>

using namespace boundary_mesh;

namespace
{
    GrowthFront makeFront(
        const std::vector<Point3> &points,
        const std::vector<Quad> &faces,
        const std::vector<SurfaceFaceId> &ids,
        std::uint32_t layer)
    {
        GrowthFront front;
        front.layer = layer;
        for (std::size_t index = 0; index < points.size(); ++index)
            front.vertices.push_back(
                {points[index], static_cast<VertexId>(index)});
        front.faces.assign(faces.begin(), faces.end());
        front.source_face_ids = ids;
        return front;
    }

    GrowthFront liftedCandidates(
        const GrowthFront &front,
        std::vector<SurfaceFaceId> retained)
    {
        GrowthFront result = front;
        result.layer = front.layer + 1;
        for (auto &vertex : result.vertices)
            vertex.position.z() += 1.0;
        result.faces.clear();
        result.source_face_ids.clear();
        for (std::size_t index = 0;
             index < front.source_face_ids.size(); ++index)
        {
            if (std::find(retained.begin(), retained.end(),
                          front.source_face_ids[index]) == retained.end())
                continue;
            result.faces.push_back(front.faces[index]);
            result.source_face_ids.push_back(front.source_face_ids[index]);
        }
        return result;
    }

    bool contains(
        const std::vector<SurfaceFaceId> &ids,
        SurfaceFaceId id)
    {
        return std::find(ids.begin(), ids.end(), id) != ids.end();
    }
}

int main()
{
    const std::vector<Point3> strip_points{
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {2,1,0}, {2,2,0}, {1,2,0}, {3,1,0}, {3,2,0},
        {0,-1,0}, {1,-1,0}};
    const std::vector<Quad> strip_faces{
        {{0,1,2,3}},       // 10: low seed
        {{2,4,5,6}},       // 11: touches a non-contact seed corner
        {{4,7,8,5}},       // 12: adjacent only to 11
        {{9,10,1,0}}};     // 20: the selected high-edge neighbor
    const std::vector<SurfaceFaceId> strip_ids{10,11,12,20};
    const GrowthFront strip = makeFront(
        strip_points, strip_faces, strip_ids, 3);
    LayerFaceSets strip_sets;
    addInitialStop(strip_sets, {10, 3, StopOrigin::Quality});
    const auto strip_result = applyCornerSuppression({
        strip, liftedCandidates(strip, {11,12,20}),
        strip_sets, 3, 1e-12});
    assert(strip_result.hasValue());
    assert((strip_result.value().removed_high_faces ==
            std::vector<SurfaceFaceId>{11}));
    assert((strip_result.value().face_sets.corner_suppression_seeds ==
            std::vector<SurfaceFaceId>{10}));
    assert((strip_result.value().face_sets.transition_low_faces ==
            std::vector<SurfaceFaceId>{10,11}));
    assert(contains(strip_result.value().retained_high_faces, 12));

    const std::vector<Point3> fan_points{
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {0,-1,0}, {1,-1,0}, {2,0,0}, {2,1,0},
        {1,2,0}, {0,2,0}, {-1,1,0}, {-1,0,0},
        {-1,2,0}};
    const std::vector<Quad> fan_faces{
        {{0,1,2,3}}, {{4,5,1,0}}, {{1,6,7,2}},
        {{2,8,9,3}}, {{3,10,11,0}}, {{3,9,12,10}}};
    const std::vector<SurfaceFaceId> fan_ids{30,31,32,33,34,35};
    const GrowthFront fan = makeFront(fan_points, fan_faces, fan_ids, 2);

    LayerFaceSets adjacent_sets;
    addInitialStop(adjacent_sets, {30, 2, StopOrigin::Collision});
    const auto adjacent = applyCornerSuppression({
        fan, liftedCandidates(fan, {31,32,35}),
        adjacent_sets, 2, 1e-12});
    assert(adjacent.hasValue());
    assert((adjacent.value().removed_high_faces ==
            std::vector<SurfaceFaceId>{35}));
    assert(contains(adjacent.value().retained_high_faces, 31));
    assert(contains(adjacent.value().retained_high_faces, 32));

    LayerFaceSets four_sets;
    addInitialStop(four_sets, {30, 2, StopOrigin::Collision});
    const auto four = applyCornerSuppression({
        fan, liftedCandidates(fan, {31,32,33,34}),
        four_sets, 2, 1e-12});
    assert(four.hasValue());
    assert(four.value().retained_high_faces.size() == 2);
    assert(four.value().removed_high_faces.size() == 2);
    for (const SurfaceFaceId removed : four.value().removed_high_faces)
    {
        assert(!contains(
            four.value().face_sets.corner_suppression_seeds, removed));
        assert(contains(
            four.value().face_sets.transition_low_faces, removed));
    }

    GrowthFront triangle_fan;
    triangle_fan.layer = 19;
    triangle_fan.vertices = {
        {{0,0,0},0}, {{1,0,0},1}, {{0,1,0},2},
        {{0,-1,0},3}, {{-1,1,0},4}, {{-1,2,0},5}};
    triangle_fan.faces = {
        Triangle{{0,1,2}}, // 7473-like stopped triangle
        Triangle{{1,0,3}}, // selected high-edge neighbour
        Triangle{{2,4,5}}  // high face at the unrelated corner
    };
    triangle_fan.source_face_ids = {7473,7438,7472};
    GrowthFront triangle_candidates = triangle_fan;
    triangle_candidates.layer = 20;
    for (auto &vertex : triangle_candidates.vertices)
        vertex.position.z() += 1.0;
    triangle_candidates.faces.erase(
        triangle_candidates.faces.begin());
    triangle_candidates.source_face_ids.erase(
        triangle_candidates.source_face_ids.begin());
    LayerFaceSets triangle_sets;
    addInitialStop(
        triangle_sets, {7473,19,StopOrigin::IsotropicStop});
    const auto triangle_suppression = applyCornerSuppression({
        triangle_fan, triangle_candidates, triangle_sets, 19, 1e-12});
    assert(triangle_suppression.hasValue());
    assert((triangle_suppression.value().removed_high_faces ==
            std::vector<SurfaceFaceId>{7472}));
    assert(contains(
        triangle_suppression.value().retained_high_faces, 7438));
    assert(!contains(
        triangle_suppression.value().face_sets.corner_suppression_seeds,
        7472));
    assert(contains(
        triangle_suppression.value().face_sets.transition_low_faces,
        7472));
}
