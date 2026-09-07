#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <boundary_mesh/io/legacy_vtk_writer.hpp>

namespace
{
    std::string fileText(const std::filesystem::path &path)
    {
        std::ifstream input(path, std::ios::binary);
        return std::string(
            std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>());
    }

    std::vector<int> cellTypes(const std::string &text)
    {
        const std::size_t position = text.find("CELL_TYPES ");
        if (position == std::string::npos) return {};
        std::istringstream input(text.substr(position));
        std::string label;
        std::size_t count = 0;
        input >> label >> count;
        std::vector<int> types(count);
        for (int &type : types) input >> type;
        return types;
    }
}

int main()
{
    using namespace boundary_mesh;

    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        "boundary_mesh_vtk_writer_test";
    std::filesystem::create_directories(root);
    const std::filesystem::path volume_path = root / "volume.vtk";
    const std::filesystem::path surface_path = root / "surface.vtk";

    VolumeMesh volume;
    volume.vertices = {
        {0.12345678901234566, 0, 0}, {1, 0, 0},
        {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1},
        {1, 1, 1}, {0, 1, 1}};
    volume.cells = {
        Tetra{{0, 1, 2, 4}},
        Pyramid{{0, 1, 2, 3, 4}},
        Prism{{0, 1, 2, 4, 5, 6}},
        Hexa{{0, 1, 2, 3, 4, 5, 6, 7}}};
    volume.metadata = {
        {CellRole::RegularLayer, 11, 1},
        {CellRole::LayerTransition, 12, 2},
        {CellRole::MultiNormalTransition, 13, 3},
        {CellRole::RegularLayer, 14, 4}};

    SurfaceMesh surface;
    surface.vertices = volume.vertices;
    surface.faces = {
        Triangle{{0, 1, 2}},
        Quad{{0, 1, 2, 3}}};
    surface.face_tags = {
        {SurfaceBoundaryKind::Wall, 1},
        {SurfaceBoundaryKind::Farfield, 2}};

    {
        std::ofstream old(volume_path);
        old << "old content";
    }
    const auto volume_status = writeLegacyVtk(volume_path, volume);
    const auto surface_status = writeLegacyVtk(surface_path, surface);
    if (!volume_status.hasValue() || !surface_status.hasValue()) return 1;
    const std::string volume_text = fileText(volume_path);
    const std::string surface_text = fileText(surface_path);
    if (volume_text.find("old content") != std::string::npos ||
        volume_text.find("ASCII") == std::string::npos ||
        volume_text.find("CELL_DATA 4\n") == std::string::npos ||
        volume_text.find(
            "SCALARS source_face_id unsigned_int 1\n"
            "LOOKUP_TABLE default\n11\n12\n13\n14\n") ==
            std::string::npos ||
        volume_text.find(
            "SCALARS layer unsigned_int 1\n"
            "LOOKUP_TABLE default\n1\n2\n3\n4\n") ==
            std::string::npos ||
        volume_text.find(
            "SCALARS cell_role int 1\n"
            "LOOKUP_TABLE default\n0\n2\n1\n0\n") ==
            std::string::npos ||
        surface_text.find("CELL_DATA") != std::string::npos ||
        volume_text.find("POINT_DATA") != std::string::npos ||
        cellTypes(volume_text) != std::vector<int>({10, 14, 13, 12}) ||
        cellTypes(surface_text) != std::vector<int>({5, 9}))
    {
        return 2;
    }

    const std::size_t points_position = volume_text.find("POINTS 8 double");
    if (points_position == std::string::npos) return 3;
    std::istringstream points(volume_text.substr(points_position));
    std::string points_label;
    std::string scalar_label;
    std::size_t point_count = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    points >> points_label >> point_count >> scalar_label >> x >> y >> z;
    if (point_count != 8 || x != volume.vertices[0].x()) return 4;

    VolumeMesh invalid = volume;
    invalid.cells.push_back(Tetra{{0, 1, 2, 99}});
    const auto invalid_status = writeLegacyVtk(root / "invalid.vtk", invalid);
    if (invalid_status.hasValue() ||
        invalid_status.error().code !=
            VtkWriteErrorCode::InvalidVertexReference ||
        invalid_status.error().cell_index != 4 ||
        invalid_status.error().vertex_id != VertexId{99})
    {
        return 5;
    }

    VolumeMesh invalid_metadata = volume;
    invalid_metadata.metadata.pop_back();
    const std::filesystem::path invalid_metadata_path =
        root / "invalid_metadata.vtk";
    {
        std::ofstream sentinel(invalid_metadata_path);
        sentinel << "preserve this content";
    }
    const auto invalid_metadata_status = writeLegacyVtk(
        invalid_metadata_path, invalid_metadata);
    if (invalid_metadata_status.hasValue() ||
        invalid_metadata_status.error().code !=
            VtkWriteErrorCode::InvalidCellMetadataCount ||
        fileText(invalid_metadata_path) != "preserve this content")
    {
        return 6;
    }

    const auto open_failure = writeLegacyVtk(
        root / "missing" / "output.vtk", surface);
    if (open_failure.hasValue() ||
        open_failure.error().code != VtkWriteErrorCode::FileOpenFailure)
    {
        return 7;
    }

    std::error_code cleanup_error;
    std::filesystem::remove_all(root, cleanup_error);
}
