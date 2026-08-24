#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    class ScopedCoutRedirect
    {
    public:
        explicit ScopedCoutRedirect(std::ostringstream &output)
            : original_(std::cout.rdbuf(output.rdbuf()))
        {
        }

        ~ScopedCoutRedirect()
        {
            std::cout.rdbuf(original_);
        }

        ScopedCoutRedirect(const ScopedCoutRedirect &) = delete;
        ScopedCoutRedirect &operator=(const ScopedCoutRedirect &) = delete;

    private:
        std::streambuf *original_;
    };

    SurfaceMesh makeMixedMesh()
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

        const VertexId offset = static_cast<VertexId>(mesh.vertices.size());
        std::vector<Point3> hexa_vertices{
            Point3{3, 0, 0}, Point3{4, 0, 0},
            Point3{4, 1, 0}, Point3{3, 1, 0},
            Point3{3, 0, 1}, Point3{4, 0, 1},
            Point3{4, 1, 1}, Point3{3, 1, 1}};
        mesh.vertices.insert(
            mesh.vertices.end(), hexa_vertices.begin(), hexa_vertices.end());
        const std::vector<SurfaceFace> hexa_faces{
            Quad{{VertexId{0}, VertexId{3}, VertexId{2}, VertexId{1}}},
            Quad{{VertexId{4}, VertexId{5}, VertexId{6}, VertexId{7}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{6}, VertexId{5}}},
            Quad{{VertexId{2}, VertexId{3}, VertexId{7}, VertexId{6}}},
            Quad{{VertexId{3}, VertexId{0}, VertexId{4}, VertexId{7}}}};
        for (const SurfaceFace &face : hexa_faces)
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
        const std::vector<SurfaceBoundaryTag> hexa_tags{
            {SurfaceBoundaryKind::Farfield, 30},
            {SurfaceBoundaryKind::Wall, 11},
            {SurfaceBoundaryKind::Farfield, 31},
            {SurfaceBoundaryKind::Farfield, 32},
            {SurfaceBoundaryKind::Farfield, 33},
            {SurfaceBoundaryKind::Farfield, 34}};
        mesh.face_tags.insert(
            mesh.face_tags.end(), hexa_tags.begin(), hexa_tags.end());
        return mesh;
    }

    bool sameCell(const VolumeCell &first, const VolumeCell &second)
    {
        return std::visit(
            [&](const auto &value)
            {
                using Cell = std::decay_t<decltype(value)>;
                const Cell *other = std::get_if<Cell>(&second);
                return other != nullptr &&
                    other->vertex_ids == value.vertex_ids;
            },
            first);
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceMesh surface = makeMixedMesh();
    const auto topology = SurfaceTopologyBuilder{}.build(surface);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(surface, topology.value());
    if (!patch.hasValue()) return 2;
    const auto front = GrowthFrontBuilder{}.buildInitial(
        surface, patch.value());
    if (!front.hasValue()) return 3;

    std::vector<SourceVertexGrowthProfile> profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        profiles.push_back(
            {vertex.source_vertex_id, {0.1, 2.0, 2}});
    }

    std::ostringstream progress_output;
    const auto result = [&]
    {
        ScopedCoutRedirect redirect(progress_output);
        return generateRegularLayers(
            surface,
            topology.value(),
            patch.value(),
            front.value(),
            profiles);
    }();
    if (!result.hasValue()) return 4;

    const std::string expected_progress =
        "generate 1 boundarylayer\n"
        "finish 1 boundarylayer. add 2 cell\n"
        "generate 2 boundarylayer\n"
        "finish 2 boundarylayer. add 2 cell\n";
    if (progress_output.str() != expected_progress)
    {
        std::cerr
            << "Unexpected progress output:\n"
            << progress_output.str();
        return 14;
    }

    const RegularLayerGrowthResult &growth = result.value();

    if (growth.mesh.vertices.size() != 21 ||
        growth.mesh.cells.size() != 4 ||
        growth.mesh.metadata.size() != 4 ||
        growth.layer_vertices.size() != 7 ||
        growth.vertices.size() != 7 ||
        growth.faces.size() != 2)
    {
        return 5;
    }

    if (std::get_if<Prism>(&growth.mesh.cells[0]) == nullptr ||
        std::get_if<Hexa>(&growth.mesh.cells[1]) == nullptr ||
        std::get_if<Prism>(&growth.mesh.cells[2]) == nullptr ||
        std::get_if<Hexa>(&growth.mesh.cells[3]) == nullptr)
    {
        return 6;
    }

    const std::vector<std::uint32_t> expected_layers{1, 1, 2, 2};
    const std::vector<SurfaceFaceId> expected_faces{
        SurfaceFaceId{1}, SurfaceFaceId{6},
        SurfaceFaceId{1}, SurfaceFaceId{6}};
    for (std::size_t index = 0; index < 4; ++index)
    {
        if (growth.mesh.metadata[index].role != CellRole::RegularLayer ||
            growth.mesh.metadata[index].layer != expected_layers[index] ||
            growth.mesh.metadata[index].source_face_id != expected_faces[index])
        {
            return 7;
        }
    }

    for (const LayerVertexRecord &record : growth.layer_vertices)
    {
        if (record.layer_vertex_ids.size() != 3) return 8;
    }
    for (const VertexGrowthRecord &record : growth.vertices)
    {
        if (record.accepted_layer_count != 2 ||
            record.profile.first_height != Scalar{0.1} ||
            record.profile.growth_ratio != Scalar{2.0} ||
            record.profile.layer_count != 2)
        {
            return 9;
        }
    }
    for (const FaceGrowthRecord &record : growth.faces)
    {
        if (record.accepted_layer_count != 2 ||
            record.status != FaceGrowthStatus::Completed ||
            record.stop_reason != FaceStopReason::VertexLayerLimit ||
            record.stop_layer != 3)
        {
            return 10;
        }
    }

    const auto repeated = generateRegularLayers(
        surface,
        topology.value(),
        patch.value(),
        front.value(),
        profiles);
    if (!repeated.hasValue() ||
        repeated.value().mesh.vertices.size() != growth.mesh.vertices.size() ||
        repeated.value().mesh.cells.size() != growth.mesh.cells.size())
    {
        return 11;
    }
    for (std::size_t index = 0; index < growth.mesh.vertices.size(); ++index)
    {
        if ((repeated.value().mesh.vertices[index] -
             growth.mesh.vertices[index]).norm() != 0.0)
        {
            return 12;
        }
    }
    for (std::size_t index = 0; index < growth.mesh.cells.size(); ++index)
    {
        if (!sameCell(growth.mesh.cells[index],
                      repeated.value().mesh.cells[index]) ||
            growth.mesh.metadata[index].source_face_id !=
                repeated.value().mesh.metadata[index].source_face_id ||
            growth.mesh.metadata[index].layer !=
                repeated.value().mesh.metadata[index].layer)
        {
            return 13;
        }
    }

    std::vector<SourceVertexGrowthProfile> isotropic_profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        isotropic_profiles.push_back(
            {vertex.source_vertex_id, {0.1, 2.0, 4}});
    }
    RegularLayerGrowthOptions isotropic_options;
    isotropic_options.isotropic_height = Scalar{0.12};
    std::ostringstream isotropic_progress;
    const auto isotropic_result = [&]
    {
        ScopedCoutRedirect redirect(isotropic_progress);
        return generateRegularLayers(
            surface,
            topology.value(),
            patch.value(),
            front.value(),
            isotropic_profiles,
            isotropic_options);
    }();
    if (!isotropic_result.hasValue()) return 15;

    const RegularLayerGrowthResult &isotropic_growth =
        isotropic_result.value();
    if (isotropic_growth.mesh.cells.size() != 3 ||
        isotropic_growth.faces.size() != 2 ||
        isotropic_growth.faces[0].accepted_layer_count != 1 ||
        isotropic_growth.faces[0].status != FaceGrowthStatus::Stopped ||
        isotropic_growth.faces[0].stop_reason !=
            FaceStopReason::IsotropicHeightReached ||
        isotropic_growth.faces[0].stop_layer != 2 ||
        isotropic_growth.faces[1].accepted_layer_count != 2 ||
        isotropic_growth.faces[1].status != FaceGrowthStatus::Stopped ||
        isotropic_growth.faces[1].stop_reason !=
            FaceStopReason::IsotropicHeightReached ||
        isotropic_growth.faces[1].stop_layer != 3)
    {
        return 16;
    }

    const std::string expected_isotropic_progress =
        "generate 1 boundarylayer\n"
        "finish 1 boundarylayer. add 2 cell\n"
        "generate 2 boundarylayer\n"
        "finish 2 boundarylayer. add 1 cell\n";
    if (isotropic_progress.str() != expected_isotropic_progress)
    {
        return 17;
    }

    bool found_triangle_interface = false;
    bool found_hexa_interface = false;
    for (const SurfaceBoundaryTag &tag :
         isotropic_growth.farfield_boundary.face_tags)
    {
        if (tag.kind != SurfaceBoundaryKind::BoundaryLayerInterface)
        {
            continue;
        }
        found_triangle_interface =
            found_triangle_interface || tag.region_id == 10;
        found_hexa_interface =
            found_hexa_interface || tag.region_id == 11;
    }
    if (!found_triangle_interface || !found_hexa_interface)
    {
        return 18;
    }

    SurfaceMesh zero_layer_surface = makeMixedMesh();
    zero_layer_surface.vertices[12].y() = Scalar{0.2};
    const auto zero_layer_topology =
        SurfaceTopologyBuilder{}.build(zero_layer_surface);
    if (!zero_layer_topology.hasValue()) return 19;
    const auto zero_layer_patch = GrowthPatchBuilder{}.build(
        zero_layer_surface,
        zero_layer_topology.value());
    if (!zero_layer_patch.hasValue()) return 20;
    const auto zero_layer_front = GrowthFrontBuilder{}.buildInitial(
        zero_layer_surface,
        zero_layer_patch.value());
    if (!zero_layer_front.hasValue()) return 21;

    std::vector<SourceVertexGrowthProfile> zero_layer_profiles;
    for (const PatchVertex &vertex : zero_layer_patch.value().vertices())
    {
        zero_layer_profiles.push_back(
            {vertex.source_vertex_id, {0.1, 1.0, 1}});
    }
    RegularLayerGrowthOptions zero_layer_options;
    zero_layer_options.cell_quality.maximum_skewness = Scalar{0.1};
    std::ostringstream zero_layer_progress;
    const auto zero_layer_result = [&]
    {
        ScopedCoutRedirect redirect(zero_layer_progress);
        return generateRegularLayers(
            zero_layer_surface,
            zero_layer_topology.value(),
            zero_layer_patch.value(),
            zero_layer_front.value(),
            zero_layer_profiles,
            zero_layer_options);
    }();
    if (!zero_layer_result.hasValue()) return 22;

    const RegularLayerGrowthResult &zero_growth =
        zero_layer_result.value();
    if (!zero_growth.mesh.cells.empty() ||
        zero_growth.faces.size() != 2 ||
        zero_growth.faces[0].accepted_layer_count != 0 ||
        zero_growth.faces[1].accepted_layer_count != 0)
    {
        return 23;
    }

    std::size_t interface_count = 0;
    bool triangle_region_found = false;
    bool quad_region_found = false;
    for (const SurfaceBoundaryTag &tag :
         zero_growth.farfield_boundary.face_tags)
    {
        if (tag.kind != SurfaceBoundaryKind::BoundaryLayerInterface)
        {
            continue;
        }
        ++interface_count;
        triangle_region_found =
            triangle_region_found || tag.region_id == 10;
        quad_region_found =
            quad_region_found || tag.region_id == 11;
    }
    if (interface_count != 2 ||
        !triangle_region_found ||
        !quad_region_found)
    {
        return 24;
    }

    return 0;
}
