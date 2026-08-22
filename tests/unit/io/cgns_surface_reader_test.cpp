#include <cassert>
#include <filesystem>
#include <fstream>
#include <variant>

#include <boundary_mesh/io/cgns_surface_reader.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

#include <helpers/cgns_fixture.hpp>

int main()
{
    using namespace boundary_mesh;

    const auto directory =
        std::filesystem::temp_directory_path() /
        "boundary_mesh_cgns_surface_reader_test";
    std::filesystem::create_directories(directory);

    const auto cgns_path = directory / "case.cgns";
    const auto map_path = directory / "case.bc.txt";
    boundary_mesh::test::writeSingleZoneMixedSurface(cgns_path);

    {
        std::ofstream map(map_path);
        map << "Wall:\n1\n";
    }

    const auto result = readCgnsSurface(cgns_path);
    assert(result.hasValue());

    const auto &mesh = result.value();
    assert(mesh.vertices.size() == 5);
    assert(mesh.faces.size() == 2);
    assert(mesh.face_tags.size() == 2);
    assert(std::holds_alternative<Triangle>(mesh.faces[0]));
    assert(std::holds_alternative<Quad>(mesh.faces[1]));
    assert(mesh.face_tags[0].kind == SurfaceBoundaryKind::Wall);
    assert(mesh.face_tags[0].region_id == 1);
    assert(mesh.face_tags[1].kind == SurfaceBoundaryKind::Wall);
    assert(mesh.face_tags[1].region_id == 1);

    const auto unicode_directory = directory / "中文路径";
    std::filesystem::create_directories(unicode_directory);
    const auto unicode_cgns = unicode_directory / "case.cgns";
    const auto unicode_map = unicode_directory / "case.bc.txt";
    std::filesystem::copy_file(
        cgns_path,
        unicode_cgns,
        std::filesystem::copy_options::overwrite_existing);
    std::filesystem::copy_file(
        map_path,
        unicode_map,
        std::filesystem::copy_options::overwrite_existing);
    const auto unicode_result = readCgnsSurface(unicode_cgns);
    assert(unicode_result.hasValue());
    assert(unicode_result.value().vertices.size() == 5);

    const auto expect_error =
        [&](const std::string &name,
            const boundary_mesh::test::SingleZoneFixtureOptions &options,
            CgnsSurfaceErrorCode expected)
        {
            const auto path = directory / (name + ".cgns");
            boundary_mesh::test::writeSingleZoneSurface(path, options);
            std::ofstream(directory / (name + ".bc.txt"))
                << "Wall:\n1\n";
            const auto failure = readCgnsSurface(path);
            assert(!failure.hasValue());
            assert(failure.error().code == expected);
        };

    {
        const auto missing = readCgnsSurface(directory / "missing.cgns");
        assert(!missing.hasValue());
        assert(missing.error().code ==
               CgnsSurfaceErrorCode::FileOpenFailure);
    }

    {
        const auto empty_path = directory / "empty.cgns";
        boundary_mesh::test::writeEmptyCgns(empty_path);
        const auto empty = readCgnsSurface(empty_path);
        assert(!empty.hasValue());
        assert(empty.error().code ==
               CgnsSurfaceErrorCode::InvalidBaseCount);
    }

    {
        const auto two_base_path = directory / "two_base.cgns";
        boundary_mesh::test::writeTwoBaseCgns(two_base_path);
        const auto two_base = readCgnsSurface(two_base_path);
        assert(!two_base.hasValue());
        assert(two_base.error().code ==
               CgnsSurfaceErrorCode::InvalidBaseCount);
    }

    {
        auto options = boundary_mesh::test::SingleZoneFixtureOptions{};
        options.cell_dimension = 3;
        expect_error(
            "dimensions",
            options,
            CgnsSurfaceErrorCode::InvalidDimensions);
    }

    {
        auto options = boundary_mesh::test::SingleZoneFixtureOptions{};
        options.zone_kind = boundary_mesh::test::FixtureZoneKind::Structured;
        expect_error(
            "structured",
            options,
            CgnsSurfaceErrorCode::InvalidZoneType);
    }

    {
        auto options = boundary_mesh::test::SingleZoneFixtureOptions{};
        options.zone_name = "not-a-number";
        expect_error(
            "invalid_zone",
            options,
            CgnsSurfaceErrorCode::InvalidZoneId);
    }

    {
        auto options = boundary_mesh::test::SingleZoneFixtureOptions{};
        options.write_coordinate_z = false;
        expect_error(
            "missing_coordinate",
            options,
            CgnsSurfaceErrorCode::MissingCoordinate);
    }

    {
        auto options = boundary_mesh::test::SingleZoneFixtureOptions{};
        options.write_non_finite_coordinate = true;
        expect_error(
            "non_finite",
            options,
            CgnsSurfaceErrorCode::NonFiniteCoordinate);
    }

    {
        auto options = boundary_mesh::test::SingleZoneFixtureOptions{};
        options.element_kind = boundary_mesh::test::FixtureElementKind::Line;
        const auto line_path = directory / "line.cgns";
        boundary_mesh::test::writeSingleZoneSurface(line_path, options);
        std::ofstream(directory / "line.bc.txt") << "Wall:\n1\n";
        const auto line = readCgnsSurface(line_path);
        assert(line.hasValue());
        assert(line.value().faces.empty());
        assert(line.value().face_tags.empty());
    }

    {
        auto options = boundary_mesh::test::SingleZoneFixtureOptions{};
        options.element_kind =
            boundary_mesh::test::FixtureElementKind::InvalidTriangleReference;
        expect_error(
            "invalid_reference",
            options,
            CgnsSurfaceErrorCode::InvalidElementReference);
    }

    {
        const auto no_map_path = directory / "no_map.cgns";
        boundary_mesh::test::writeSingleZoneMixedSurface(no_map_path);
        const auto no_map = readCgnsSurface(no_map_path);
        assert(!no_map.hasValue());
        assert(no_map.error().code ==
               CgnsSurfaceErrorCode::BoundaryMapFailure);
    }

    std::filesystem::remove_all(directory);
    return 0;
}
