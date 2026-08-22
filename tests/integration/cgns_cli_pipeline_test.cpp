#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <cli/boundary_mesh_command.hpp>

#include <helpers/cgns_fixture.hpp>

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
    expect_argument_error({
        "--input", "a.cgns", "--input", "b.cgns",
        "--first-height", "0.1", "--growth-ratio", "1",
        "--layer-count", "1"});
    expect_argument_error({
        "--input", "a.cgns", "--first-height", "nan",
        "--growth-ratio", "1", "--layer-count", "1"});
    expect_argument_error({
        "--input", "a.cgns", "--first-height", "0.1",
        "--growth-ratio", "1", "--layer-count", "4294967296"});

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
        "--output-prefix", prefix.string()};
    std::ostringstream output;
    std::ostringstream error;
    const int code = boundary_mesh::runBoundaryMeshCommand(
        arguments,
        output,
        error);

    assert(code == 0);
    assert(error.str().empty());
    assert(output.str().find("maximum_skewness=0.95") !=
           std::string::npos);
    assert(output.str().find("max_neighbor_layer_difference=1") !=
           std::string::npos);

    const auto volume_path =
        std::filesystem::path(prefix.string() + "_boundary_layer.vtk");
    const auto surface_path =
        std::filesystem::path(prefix.string() + "_farfield_boundary.vtk");
    assert(std::filesystem::file_size(volume_path) > 0);
    assert(std::filesystem::file_size(surface_path) > 0);

    std::filesystem::remove_all(directory);
    return 0;
}
