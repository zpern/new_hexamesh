#include <array>

#include <boundary_mesh/growth/multi_normal_types.hpp>

int main()
{
    using namespace boundary_mesh;

    MultiNormalTransitionResult result;
    result.omitted_quad_transitions.push_back(
        OmittedQuadTransition{
            SurfaceFaceId{7},
            {VertexId{4}, VertexId{5}, VertexId{6}, VertexId{8}},
            {VertexId{1}, VertexId{2}, VertexId{3}, VertexId{4}},
            {true, false, true, false}});

    if (result.omitted_quad_transitions[0].source_face_id !=
            SurfaceFaceId{7} ||
        !result.omitted_quad_transitions[0].moved_corners[0] ||
        result.omitted_quad_transitions[0].moved_corners[1])
    {
        return 1;
    }

    const GrowthFrontVertex ordinary{Point3{1, 2, 3}, VertexId{9}};
    if (ordinary.branch_id != 0 || ordinary.multi_normal_branch)
    {
        return 2;
    }

    return 0;
}
