#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <cli/boundary_mesh_command.hpp>

#include <helpers/cgns_fixture.hpp>

namespace
{
    std::string fileText(const std::filesystem::path &path)
    {
        std::ifstream input(path, std::ios::binary);
        return std::string(
            std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>());
    }
}

int main()
{
    const auto expect_argument_error =
        [](const std::vector<std::string> &arguments)
    {
        std::ostringstream output;
        std::ostringstream error;
        assert(boundary_mesh::runBoundaryMeshCommand(
                   arguments,
                   output,
                   error) == 2);
        assert(output.str().empty());
        assert(error.str().find("usage:") != std::string::npos);
    };
    expect_argument_error({});
    expect_argument_error({"--unknown", "1"});
    expect_argument_error({"--input", "a.cgns", "--input", "b.cgns",
                           "--first-height", "0.1", "--growth-ratio", "1",
                           "--layer-count", "1"});
    expect_argument_error({"--input", "a.cgns", "--first-height", "nan",
                           "--growth-ratio", "1", "--layer-count", "1"});
    expect_argument_error({"--input", "a.cgns", "--first-height", "0.1",
                           "--growth-ratio", "1", "--layer-count", "4294967296"});
    expect_argument_error({"--input", "a.cgns", "--first-height", "0.1",
                           "--growth-ratio", "1", "--layer-count", "1",
                           "--isotropic-height", "0"});
    expect_argument_error({"--input", "a.cgns", "--first-height", "0.1",
                           "--growth-ratio", "1", "--layer-count", "1",
                           "--isotropic-height", "-1"});
    expect_argument_error({"--input", "a.cgns", "--first-height", "0.1",
                           "--growth-ratio", "1", "--layer-count", "1",
                           "--isotropic-height", "nan"});
    expect_argument_error({"--input", "a.cgns", "--first-height", "0.1",
                           "--growth-ratio", "1", "--layer-count", "1",
                           "--isotropic-height", "inf"});
    expect_argument_error({"--input", "a.cgns", "--first-height", "0.1",
                           "--growth-ratio", "1", "--layer-count", "1",
                           "--debuglog", "maybe"});

    const auto directory =
        std::filesystem::temp_directory_path() /
        "boundary_mesh_cgns_cli_pipeline_test";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);

    const auto input = directory / "cube.cgns";
    boundary_mesh::test::writeClosedCubeSurface(input);
    std::ofstream(directory / "cube.bc.txt")
        << "Far:\n2\n\nWall:\n1\n";

    const auto prefix = directory / "output" / "cube";
    const std::vector<std::string> arguments{
        "--input", input.string(),
        "--first-height", "0.1",
        "--growth-ratio", "1.0",
        "--layer-count", "1",
        "--multi-normal", "true",
        "--output-prefix", prefix.string()};
    std::ostringstream output;
    std::ostringstream error;
    const int code = boundary_mesh::runBoundaryMeshCommand(
        arguments,
        output,
        error);

    assert(code == 0);
    assert(error.str().empty());
    assert(output.str().find("Generating multi-normal boundary layer") !=
           std::string::npos);
    assert(output.str().find("wall_regions=1") != std::string::npos);
    assert(output.str().find("far_regions=2") != std::string::npos);
    assert(output.str().find("wall_faces=1") != std::string::npos);
    assert(output.str().find("symmetry_regions=") == std::string::npos);
    assert(output.str().find("internal_regions=") == std::string::npos);
    assert(output.str().find("input_vertices=") == std::string::npos);
    assert(output.str().find("volume_cells=") == std::string::npos);
    assert(output.str().find("stop_none=") == std::string::npos);
    assert(!std::filesystem::exists(
        std::filesystem::path(prefix.string() + "_debug.txt")));

    const auto debug_prefix = directory / "output" / "cube-debug";
    const std::vector<std::string> debug_arguments{
        "--input", input.string(),
        "--first-height", "0.1",
        "--growth-ratio", "1.0",
        "--layer-count", "1",
        "--debuglog", "true",
        "--output-prefix", debug_prefix.string()};
    std::ostringstream debug_output;
    std::ostringstream debug_error;
    assert(boundary_mesh::runBoundaryMeshCommand(
               debug_arguments, debug_output, debug_error) == 0);
    assert(debug_error.str().empty());
    const auto debug_path = std::filesystem::path(
        debug_prefix.string() + "_debug.txt");
    assert(std::filesystem::exists(debug_path));
    const std::string debug_text = fileText(debug_path);
    assert(debug_text.rfind("0 ", 0) == 0);
    assert(debug_text.find(" accepted_layers=") != std::string::npos);
    assert(debug_text.find(" stop_layer=") != std::string::npos);

    const auto isotropic_prefix = directory / "output" / "cube-isotropic";
    const std::vector<std::string> isotropic_arguments{
        "--input", input.string(),
        "--first-height", "0.1",
        "--growth-ratio", "1.0",
        "--layer-count", "1",
        "--isotropic-height", "0.75",
        "--output-prefix", isotropic_prefix.string()};
    std::ostringstream isotropic_output;
    std::ostringstream isotropic_error;
    if (boundary_mesh::runBoundaryMeshCommand(
            isotropic_arguments,
            isotropic_output,
            isotropic_error) != 0 ||
        !isotropic_error.str().empty() ||
        isotropic_output.str().find("wall_faces=1") ==
            std::string::npos)
    {
        return 22;
    }

    const auto volume_path =
        std::filesystem::path(prefix.string() + "_boundary_layer.vtk");
    const auto surface_path =
        std::filesystem::path(prefix.string() + "_farfield_boundary.vtk");
    const auto top_path =
        std::filesystem::path(prefix.string() + "_boundary_layer_top.vtk");
    assert(std::filesystem::file_size(volume_path) > 0);
    assert(std::filesystem::file_size(surface_path) > 0);
    assert(std::filesystem::file_size(top_path) > 0);

    const auto reversed_input = directory / "cube-reversed.cgns";
    boundary_mesh::test::writeClosedCubeSurface(
        reversed_input,
        true,
        true);
    std::ofstream(directory / "cube-reversed.bc.txt")
        << "Far:\n2\n\nWall:\n1\n";
    const auto reversed_prefix = directory / "output" / "cube-reversed";
    const std::vector<std::string> reversed_arguments{
        "--input", reversed_input.string(),
        "--first-height", "0.1",
        "--growth-ratio", "1.0",
        "--layer-count", "1",
        "--output-prefix", reversed_prefix.string()};
    std::ostringstream reversed_output;
    std::ostringstream reversed_error;
    assert(boundary_mesh::runBoundaryMeshCommand(
               reversed_arguments,
               reversed_output,
               reversed_error) == 0);
    assert(reversed_error.str().empty());

    const auto reversed_volume_path = std::filesystem::path(
        reversed_prefix.string() + "_boundary_layer.vtk");
    const auto reversed_surface_path = std::filesystem::path(
        reversed_prefix.string() + "_farfield_boundary.vtk");
    assert(fileText(reversed_volume_path) == fileText(volume_path));
    assert(fileText(reversed_surface_path) == fileText(surface_path));

    std::filesystem::remove_all(directory);
    return 0;
}
