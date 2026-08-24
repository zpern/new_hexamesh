#include <cmath>

#include <boundary_mesh/growth/multi_normal_split_planner.hpp>

namespace
{
    using namespace boundary_mesh;

    constexpr Scalar pi = 3.14159265358979323846;

    Vector3 rotated(Scalar degrees)
    {
        const Scalar angle = degrees * pi / Scalar{180};
        return Vector3{std::sin(angle), 0, std::cos(angle)};
    }

    GrowthFront fanFront()
    {
        GrowthFront front;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{100}},
            {Point3{1, 0, 0}, VertexId{101}},
            {Point3{0, 1, 0}, VertexId{102}},
            {Point3{-1, 0, 0}, VertexId{103}},
            {Point3{0, -1, 0}, VertexId{104}}};
        return front;
    }

    IncidentFaceFan fan(const std::vector<Vector3> &normals)
    {
        IncidentFaceFan result;
        result.center_vertex = 0;
        result.closed = true;
        for (std::size_t i = 0; i < normals.size(); ++i)
        {
            result.sectors.push_back(IncidentFaceSector{
                i,
                static_cast<VertexId>(i + 1),
                static_cast<VertexId>((i + 1) % normals.size() + 1),
                normals[i]});
        }
        return result;
    }

    std::vector<IncidentFaceFan> fans(const IncidentFaceFan &center)
    {
        std::vector<IncidentFaceFan> result(5);
        for (std::size_t i = 0; i < result.size(); ++i)
        {
            result[i].center_vertex = static_cast<VertexId>(i);
        }
        result[0] = center;
        return result;
    }
}

int main()
{
    using namespace boundary_mesh;

    const GrowthFront front = fanFront();
    MultiNormalOptions options;
    options.enabled = true;
    options.split_skewness_threshold = 0.80;

    const auto planar = planMultiNormalSplits(
        front,
        fans(fan({Vector3::UnitZ(), Vector3::UnitZ(),
                  Vector3::UnitZ(), Vector3::UnitZ()})),
        options);
    if (!planar.hasValue() || !planar.value().empty()) return 1;

    const Vector3 far = rotated(160);
    const auto sharp = planMultiNormalSplits(
        front,
        fans(fan({Vector3::UnitZ(), Vector3::UnitZ(), far, far})),
        options);
    if (!sharp.hasValue() || sharp.value().size() != 1) return 2;
    const VertexSplitPlan &plan = sharp.value().front();
    if (plan.source_vertex_id != VertexId{100} ||
        plan.branches.size() != 2 || plan.splitter_neighbors.size() != 2)
    {
        return 3;
    }
    if (plan.branches[0].face_indices.empty() ||
        plan.branches[1].face_indices.empty())
    {
        return 4;
    }
    for (const SplitBranch &branch : plan.branches)
    {
        if (!branch.direction.allFinite() ||
            std::abs(branch.direction.norm() - Scalar{1}) > Scalar{1e-12})
        {
            return 5;
        }
    }
    if (!(plan.selected_skewness < plan.original_skewness)) return 6;

    return 0;
}
