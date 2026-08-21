#include <array>
#include <cmath>
#include <limits>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makePrismMesh()
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
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Symmetry, 30},
            {SurfaceBoundaryKind::Symmetry, 31},
            {SurfaceBoundaryKind::Symmetry, 32}};
        return mesh;
    }

    Result<GrowthPatch, GrowthPatchError> makePatch()
    {
        const SurfaceMesh mesh = makePrismMesh();
        const auto topology = SurfaceTopologyBuilder{}.build(mesh);
        if (!topology.hasValue())
        {
            return Result<GrowthPatch, GrowthPatchError>::failure(
                EmptyGrowthPatch{});
        }
        return GrowthPatchBuilder{}.build(mesh, topology.value());
    }
}

int main()
{
    using namespace boundary_mesh;

    const auto patch_result = makePatch();
    if (!patch_result.hasValue()) return 1;
    const GrowthPatch &patch = patch_result.value();

    const std::vector<SourceVertexGrowthProfile> valid{
        {VertexId{0}, {0.1, 1.0, 0}},
        {VertexId{1}, {0.2, 2.0, 3}},
        {VertexId{2}, {0.3, 0.5, 4}}};

    const auto table = GrowthProfileBuilder{}.build(patch, valid);
    if (!table.hasValue()) return 2;
    const auto *profile = table.value().find(VertexId{1});
    if (profile == nullptr || profile->layer_count != 3) return 3;

    const auto height1 = table.value().height(VertexId{1}, 1);
    const auto height3 = table.value().height(VertexId{1}, 3);
    if (!height1.hasValue() || !height3.hasValue() ||
        std::abs(height1.value() - 0.2) > 1e-12 ||
        std::abs(height3.value() - 0.8) > 1e-12)
    {
        return 4;
    }

    auto missing = valid;
    missing.pop_back();
    const auto missing_result = GrowthProfileBuilder{}.build(patch, missing);
    const auto *missing_error = missing_result.hasValue()
        ? nullptr
        : std::get_if<MissingVertexGrowthProfile>(&missing_result.error());
    if (missing_error == nullptr ||
        missing_error->source_vertex_id != VertexId{2})
    {
        return 5;
    }

    auto duplicate = valid;
    duplicate.push_back(valid.front());
    const auto duplicate_result = GrowthProfileBuilder{}.build(
        patch, duplicate);
    const auto *duplicate_error = duplicate_result.hasValue()
        ? nullptr
        : std::get_if<DuplicateVertexGrowthProfile>(
              &duplicate_result.error());
    if (duplicate_error == nullptr ||
        duplicate_error->source_vertex_id != VertexId{0})
    {
        return 6;
    }

    auto unknown = valid;
    unknown.push_back({VertexId{99}, {0.1, 1.0, 1}});
    const auto unknown_result = GrowthProfileBuilder{}.build(patch, unknown);
    const auto *unknown_error = unknown_result.hasValue()
        ? nullptr
        : std::get_if<UnknownVertexGrowthProfile>(&unknown_result.error());
    if (unknown_error == nullptr ||
        unknown_error->source_vertex_id != VertexId{99})
    {
        return 7;
    }

    const std::array<Scalar, 3> bad_heights{
        Scalar{0},
        std::numeric_limits<Scalar>::quiet_NaN(),
        std::numeric_limits<Scalar>::infinity()};
    for (const Scalar value : bad_heights)
    {
        auto invalid = valid;
        invalid[0].profile.first_height = value;
        const auto result = GrowthProfileBuilder{}.build(patch, invalid);
        const auto *error = result.hasValue()
            ? nullptr
            : std::get_if<InvalidFirstHeight>(&result.error());
        if (error == nullptr || error->source_vertex_id != VertexId{0})
        {
            return 8;
        }
    }

    const std::array<Scalar, 3> bad_ratios{
        Scalar{0},
        std::numeric_limits<Scalar>::quiet_NaN(),
        std::numeric_limits<Scalar>::infinity()};
    for (const Scalar value : bad_ratios)
    {
        auto invalid = valid;
        invalid[0].profile.growth_ratio = value;
        const auto result = GrowthProfileBuilder{}.build(patch, invalid);
        const auto *error = result.hasValue()
            ? nullptr
            : std::get_if<InvalidGrowthRatio>(&result.error());
        if (error == nullptr || error->source_vertex_id != VertexId{0})
        {
            return 9;
        }
    }

    auto overflowing = valid;
    overflowing[0].profile.first_height =
        std::numeric_limits<Scalar>::max();
    overflowing[0].profile.growth_ratio = Scalar{2};
    const auto overflow_table = GrowthProfileBuilder{}.build(
        patch, overflowing);
    if (!overflow_table.hasValue()) return 10;
    const auto overflow_height = overflow_table.value().height(
        VertexId{0}, 2);
    const auto *overflow_error = overflow_height.hasValue()
        ? nullptr
        : &overflow_height.error();
    if (overflow_error == nullptr ||
        overflow_error->source_vertex_id != VertexId{0} ||
        overflow_error->layer != 2)
    {
        return 11;
    }

    return 0;
}
