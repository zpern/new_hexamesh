#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <vector>

#include "../support/boundary_meshing_fixture.hpp"
#include <boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/io/legacy_vtk_writer.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

using namespace boundary_mesh;
using namespace boundary_mesh::test;

namespace
{
    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError> generateCase(
        const BoundaryMeshingFixture &fixture,
        SurfaceBoundaryKind sliding_kind)
    {
        SurfaceMesh mesh = fixture.mesh;
        const auto topology = SurfaceTopologyBuilder{}.build(mesh);
        if (!topology.hasValue())
            throw std::runtime_error("Failed to build real-case topology");
        for (SurfaceBoundaryTag &tag : mesh.face_tags)
            if (tag.region_id == 3) tag.kind = sliding_kind;

        const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
        if (!patch.hasValue())
            throw std::runtime_error("Failed to build real-case growth patch");
        const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
        if (!front.hasValue())
            throw std::runtime_error("Failed to build real-case growth front");

        std::vector<SourceVertexGrowthProfile> profiles;
        profiles.reserve(patch.value().vertices().size());
        for (const PatchVertex &vertex : patch.value().vertices())
            profiles.push_back({vertex.source_vertex_id,
                {fixture.first_height, fixture.growth_ratio,
                 fixture.layer_count}});

        RegularLayerGrowthOptions options;
        options.cell_quality.maximum_skewness =
            fixture.maximum_prism_skewness;
        options.isotropic_height = fixture.isotropic_stop;
        options.max_layer_diff = 6;
        return generateIncrementalBoundaryLayers(
            mesh, topology.value(), patch.value(), front.value(),
            profiles, options);
    }

    bool sameDecisions(
        const RegularLayerGrowthResult &left,
        const RegularLayerGrowthResult &right)
    {
        if (left.faces.size() != right.faces.size()) return false;
        for (std::size_t i = 0; i < left.faces.size(); ++i)
        {
            const FaceGrowthRecord &a = left.faces[i];
            const FaceGrowthRecord &b = right.faces[i];
            if (a.source_face_id != b.source_face_id ||
                a.accepted_layer_count != b.accepted_layer_count ||
                a.status != b.status || a.stop_reason != b.stop_reason ||
                a.stop_layer != b.stop_layer)
                return false;
        }
        return true;
    }
}

int main()
{
    try
    {
    const auto fixture = readBoundaryMeshingFixture(
        std::filesystem::path(BOUNDARY_MESH_SOURCE_DIR) /
        "tests/data/symm_intersection/BoundaryMeshing.txt");
    if (fixture.mesh.vertices.size() != 4298 ||
        fixture.mesh.faces.size() != 8592 ||
        fixture.layer_count != 20 ||
        std::abs(fixture.first_height - Scalar{0.100000001}) > 1e-14 ||
        std::abs(fixture.growth_ratio - Scalar{1.2}) > 1e-14 ||
        fixture.maximum_prism_skewness != Scalar{1} ||
        fixture.maximum_pyramid_skewness != Scalar{1} ||
        fixture.maximum_ratio_difference != Scalar{0.25} ||
        fixture.isotropic_stop != Scalar{1} ||
        fixture.use_multiple_normals ||
        fixture.face_parameters.size() != 6 ||
        fixture.face_parameters.at(3).layer_count != 0 ||
        fixture.face_parameters.at(0).layer_count != 20)
        return 1;

    std::size_t symmetry_faces{};
    std::size_t wall_faces{};
    for (const SurfaceBoundaryTag tag : fixture.mesh.face_tags)
    {
        symmetry_faces += tag.kind == SurfaceBoundaryKind::Symmetry;
        wall_faces += tag.kind == SurfaceBoundaryKind::Wall;
    }
    if (symmetry_faces == 0 || wall_faces == 0) return 2;

    const auto symmetry = generateCase(fixture, SurfaceBoundaryKind::Symmetry);
    if (!symmetry.hasValue() || symmetry.value().mesh.cells.empty()) return 4;
    const auto internal = generateCase(fixture, SurfaceBoundaryKind::Internal);
    if (!internal.hasValue() || internal.value().mesh.cells.empty()) return 5;
    if (!sameDecisions(symmetry.value(), internal.value())) return 6;

    const auto transition_count = [](const RegularLayerGrowthResult &result)
    {
        return std::count_if(
            result.mesh.metadata.begin(), result.mesh.metadata.end(),
            [](const CellMetadata &metadata)
            { return metadata.role == CellRole::LayerTransition; });
    };
    if (transition_count(symmetry.value()) == 0 ||
        transition_count(internal.value()) == 0)
        return 9;

    std::size_t collision_stops{};
    for (const FaceGrowthRecord &face : symmetry.value().faces)
        collision_stops += face.stop_reason == FaceStopReason::Collision;
    if (collision_stops == 0) return 7;

    const auto symmetry_path = std::filesystem::current_path() /
        "symm_intersection_symmetry_boundary_layer.vtk";
    const auto internal_path = std::filesystem::current_path() /
        "symm_intersection_internal_boundary_layer.vtk";
    if (!writeLegacyVtk(symmetry_path, symmetry.value().mesh).hasValue() ||
        !writeLegacyVtk(internal_path, internal.value().mesh).hasValue() ||
        std::filesystem::file_size(symmetry_path) == 0 ||
        std::filesystem::file_size(internal_path) == 0)
        return 8;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 3;
    }
}
