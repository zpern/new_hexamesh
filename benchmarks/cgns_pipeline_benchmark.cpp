#include <chrono>
#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/io/cgns_surface_reader.hpp>
#include <boundary_mesh/io/legacy_vtk_writer.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using Clock = std::chrono::steady_clock;

    double seconds(Clock::time_point first, Clock::time_point second)
    {
        return std::chrono::duration<double>(second - first).count();
    }

    std::uint64_t peakWorkingSet()
    {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS counters{};
        if (GetProcessMemoryInfo(
                GetCurrentProcess(),
                &counters,
                sizeof(counters)) == 0)
        {
            return 0;
        }
        return static_cast<std::uint64_t>(
            counters.PeakWorkingSetSize);
#else
        return 0;
#endif
    }

    std::array<std::size_t, 8> stopReasonCounts(
        const boundary_mesh::RegularLayerGrowthResult &growth)
    {
        std::array<std::size_t, 8> counts{};
        for (const boundary_mesh::FaceGrowthRecord &face : growth.faces)
        {
            ++counts[static_cast<std::size_t>(face.stop_reason)];
        }
        return counts;
    }
}

int main(int argc, char **argv)
{
    using namespace boundary_mesh;

    if (argc < 2 || argc > 3)
    {
        std::cerr << "usage: boundary_mesh_cgns_pipeline_benchmark "
                  << "FILE.cgns [OUTPUT_PREFIX]\n";
        return 2;
    }

    const std::filesystem::path input(argv[1]);
    const std::filesystem::path prefix = argc == 3
        ? std::filesystem::path(argv[2])
        : input.parent_path() / input.stem();
    std::error_code directory_error;
    if (!prefix.parent_path().empty())
    {
        std::filesystem::create_directories(
            prefix.parent_path(),
            directory_error);
    }
    if (directory_error)
    {
        std::cerr << "failed to create output directory\n";
        return 1;
    }

    const auto total_start = Clock::now();
    const auto read_start = total_start;
    const auto surface = readCgnsSurface(input);
    const auto read_end = Clock::now();
    if (!surface.hasValue())
    {
        std::cerr << "read failed code="
                  << static_cast<int>(surface.error().code) << '\n';
        return 1;
    }

    const auto topology_start = read_end;
    const auto topology =
        SurfaceTopologyBuilder{}.build(surface.value());
    if (!topology.hasValue())
    {
        std::cerr << "topology failed\n";
        return 1;
    }
    const auto patch = GrowthPatchBuilder{}.build(
        surface.value(),
        topology.value());
    if (!patch.hasValue())
    {
        std::cerr << "patch failed\n";
        return 1;
    }
    const auto front = GrowthFrontBuilder{}.buildInitial(
        surface.value(),
        patch.value());
    if (!front.hasValue())
    {
        std::cerr << "front failed\n";
        return 1;
    }
    const auto topology_end = Clock::now();

    std::vector<SourceVertexGrowthProfile> profiles;
    profiles.reserve(patch.value().vertices().size());
    for (const auto &vertex : patch.value().vertices())
    {
        profiles.push_back({
            vertex.source_vertex_id,
            {0.1, 1.0, 1}});
    }
    RegularLayerGrowthOptions options;
    options.cell_quality.maximum_skewness = 0.95;
    options.max_neighbor_layer_difference = 1;

    RegularLayerGrowthOptions baseline_options = options;
    baseline_options.field_smoothing.skewness.enabled = false;
    const auto baseline_growth_start = topology_end;
    const auto baseline_growth = generateRegularLayers(
        surface.value(),
        topology.value(),
        patch.value(),
        front.value(),
        profiles,
        baseline_options);
    const auto baseline_growth_end = Clock::now();
    if (!baseline_growth.hasValue())
    {
        std::cerr << "baseline growth failed\n";
        return 1;
    }

    const auto growth_start = baseline_growth_end;
    const auto growth = generateRegularLayers(
        surface.value(),
        topology.value(),
        patch.value(),
        front.value(),
        profiles,
        options);
    const auto growth_end = Clock::now();
    if (!growth.hasValue())
    {
        std::cerr << "growth failed\n";
        return 1;
    }

    const auto write_start = growth_end;
    const auto volume_status = writeLegacyVtk(
        std::filesystem::path(
            prefix.string() + "_boundary_layer.vtk"),
        growth.value().mesh);
    const auto surface_status = writeLegacyVtk(
        std::filesystem::path(
            prefix.string() + "_farfield_boundary.vtk"),
        growth.value().farfield_boundary);
    const auto write_end = Clock::now();
    if (!volume_status.hasValue() || !surface_status.hasValue())
    {
        std::cerr << "write failed\n";
        return 1;
    }

    const auto peak = peakWorkingSet();
    const auto baseline_stop_counts =
        stopReasonCounts(baseline_growth.value());
    const auto stop_counts = stopReasonCounts(growth.value());
    std::cout << "read_seconds=" << seconds(read_start, read_end) << '\n'
              << "topology_seconds="
              << seconds(topology_start, topology_end) << '\n'
              << "growth_seconds="
              << seconds(growth_start, growth_end) << '\n'
              << "baseline_growth_seconds="
              << seconds(baseline_growth_start, baseline_growth_end)
              << '\n'
              << "write_seconds="
              << seconds(write_start, write_end) << '\n'
              << "total_seconds="
              << seconds(total_start, write_end) << '\n'
              << "peak_working_set_bytes=" << peak << '\n'
              << "volume_cells=" << growth.value().mesh.cells.size()
              << '\n'
              << "farfield_faces="
              << growth.value().farfield_boundary.faces.size()
              << '\n'
              << "stop_none=" << stop_counts[0] << '\n'
              << "stop_vertex_layer_limit=" << stop_counts[1] << '\n'
              << "stop_degenerate_candidate=" << stop_counts[2] << '\n'
              << "stop_reversed_candidate=" << stop_counts[3] << '\n'
              << "stop_locally_inverted_candidate=" << stop_counts[4] << '\n'
              << "stop_skewness_exceeded=" << stop_counts[5] << '\n'
              << "stop_collision=" << stop_counts[6] << '\n'
              << "stop_neighbor_layer_constraint=" << stop_counts[7] << '\n';

    const auto &smoothing = growth.value().smoothing_diagnostics;
    std::cout << "baseline_stop_skewness_exceeded="
              << baseline_stop_counts[5] << '\n'
              << "smoothing_activated_vertices="
              << smoothing.activated_vertices << '\n'
              << "smoothing_updated_vertices="
              << smoothing.updated_vertices << '\n'
              << "smoothing_maximum_skewness_before="
              << smoothing.maximum_skewness_before << '\n'
              << "smoothing_maximum_skewness_after="
              << smoothing.maximum_skewness_after << '\n';

#ifdef _WIN32
    constexpr std::uint64_t one_gibibyte = 1024ULL * 1024ULL * 1024ULL;
    if (peak > one_gibibyte)
    {
        return 3;
    }
#endif
    return 0;
}
