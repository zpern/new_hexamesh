#include <cli/boundary_mesh_command.hpp>

#include <array>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <locale>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/io/cgns_surface_reader.hpp>
#include <boundary_mesh/io/legacy_vtk_writer.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace boundary_mesh
{
    namespace
    {
        enum class ParseStatus
        {
            Success,
            Failure
        };

        bool parseUnsigned(
            const std::string &text,
            std::uint32_t &value)
        {
            if (text.empty())
            {
                return false;
            }
            const auto result = std::from_chars(
                text.data(),
                text.data() + text.size(),
                value);
            return result.ec == std::errc{} &&
                   result.ptr == text.data() + text.size();
        }

        bool parseScalar(
            const std::string &text,
            Scalar &value)
        {
            std::istringstream stream(text);
            stream.imbue(std::locale::classic());
            stream >> std::noskipws >> value;
            return stream && stream.eof() && std::isfinite(value);
        }

        ParseStatus parseArguments(
            const std::vector<std::string> &arguments,
            BoundaryMeshCommandOptions &options,
            std::string &message)
        {
            std::unordered_set<std::string> seen;
            bool has_input = false;
            bool has_first_height = false;
            bool has_growth_ratio = false;
            bool has_layer_count = false;

            for (std::size_t index = 0;
                 index < arguments.size();
                 index += 2)
            {
                if (index + 1 >= arguments.size())
                {
                    message = "missing value for " + arguments[index];
                    return ParseStatus::Failure;
                }

                const auto &name = arguments[index];
                const auto &value = arguments[index + 1];
                if (!seen.insert(name).second)
                {
                    message = "duplicate argument: " + name;
                    return ParseStatus::Failure;
                }

                if (name == "--input")
                {
                    options.input = value;
                    has_input = !value.empty();
                }
                else if (name == "--first-height")
                {
                    has_first_height =
                        parseScalar(value, options.first_height) &&
                        options.first_height > Scalar{0};
                }
                else if (name == "--growth-ratio")
                {
                    has_growth_ratio =
                        parseScalar(value, options.growth_ratio) &&
                        options.growth_ratio > Scalar{0};
                }
                else if (name == "--layer-count")
                {
                    has_layer_count =
                        parseUnsigned(value, options.layer_count) &&
                        options.layer_count > 0;
                }
                else if (name == "--maximum-skewness")
                {
                    if (!parseScalar(value, options.maximum_skewness) ||
                        options.maximum_skewness < Scalar{0} ||
                        options.maximum_skewness > Scalar{1})
                    {
                        message = "invalid maximum skewness";
                        return ParseStatus::Failure;
                    }
                }
                else if (name == "--max-neighbor-layer-difference")
                {
                    if (!parseUnsigned(
                            value,
                            options.max_neighbor_layer_difference))
                    {
                        message = "invalid maximum neighbor layer difference";
                        return ParseStatus::Failure;
                    }
                }
                else if (name == "--isotropic-height")
                {
                    if (!parseScalar(value, options.isotropic_height) ||
                        options.isotropic_height <= Scalar{0})
                    {
                        message = "invalid isotropic height";
                        return ParseStatus::Failure;
                    }
                }
                else if (name == "--output-prefix")
                {
                    options.output_prefix = value;
                    if (value.empty())
                    {
                        message = "empty output prefix";
                        return ParseStatus::Failure;
                    }
                }
                else
                {
                    message = "unknown argument: " + name;
                    return ParseStatus::Failure;
                }
            }

            if (!has_input || !has_first_height ||
                !has_growth_ratio || !has_layer_count)
            {
                message = "missing or invalid required argument";
                return ParseStatus::Failure;
            }
            if (options.output_prefix.empty())
            {
                options.output_prefix =
                    options.input.parent_path() /
                    options.input.stem();
            }
            return ParseStatus::Success;
        }

        void printUsage(std::ostream &error)
        {
            error
                << "usage: boundary_mesh_cli --input FILE "
                << "--first-height VALUE --growth-ratio VALUE "
                << "--layer-count COUNT [--maximum-skewness VALUE] "
                << "[--max-neighbor-layer-difference COUNT] "
                << "[--isotropic-height VALUE] "
                << "[--output-prefix PATH]\n";
        }

        std::array<std::size_t, 9> stopReasonCounts(
            const RegularLayerGrowthResult &growth)
        {
            std::array<std::size_t, 9> counts{};
            for (const FaceGrowthRecord &face : growth.faces)
            {
                ++counts[static_cast<std::size_t>(face.stop_reason)];
            }
            return counts;
        }

        void printStopReasonCounts(
            std::ostream &output,
            const RegularLayerGrowthResult &growth)
        {
            const auto counts = stopReasonCounts(growth);
            output << "stop_none=" << counts[0] << '\n'
                   << "stop_vertex_layer_limit=" << counts[1] << '\n'
                   << "stop_degenerate_candidate=" << counts[2] << '\n'
                   << "stop_reversed_candidate=" << counts[3] << '\n'
                   << "stop_locally_inverted_candidate=" << counts[4] << '\n'
                   << "stop_skewness_exceeded=" << counts[5] << '\n'
                   << "stop_collision=" << counts[6] << '\n'
                   << "stop_neighbor_layer_constraint=" << counts[7] << '\n'
                   << "stop_isotropic_height=" << counts[8] << '\n';
        }
    }

    int runBoundaryMeshCommand(
        const std::vector<std::string> &arguments,
        std::ostream &output,
        std::ostream &error)
    {
        BoundaryMeshCommandOptions command_options;
        std::string parse_message;
        if (parseArguments(
                arguments,
                command_options,
                parse_message) != ParseStatus::Success)
        {
            error << parse_message << '\n';
            printUsage(error);
            return 2;
        }

        const auto surface = readCgnsSurface(command_options.input);
        if (!surface.hasValue())
        {
            error << "failed to read CGNS surface\n";
            return 3;
        }

        const auto topology =
            SurfaceTopologyBuilder{}.build(surface.value());
        if (!topology.hasValue())
        {
            error << "failed to build surface topology\n";
            return 4;
        }

        const auto patch = GrowthPatchBuilder{}.build(
            surface.value(),
            topology.value());
        if (!patch.hasValue())
        {
            error << "failed to build growth patch\n";
            return 5;
        }

        const auto front = GrowthFrontBuilder{}.buildInitial(
            surface.value(),
            patch.value());
        if (!front.hasValue())
        {
            error << "failed to build growth front\n";
            return 5;
        }

        std::vector<SourceVertexGrowthProfile> profiles;
        profiles.reserve(patch.value().vertices().size());
        for (const auto &vertex : patch.value().vertices())
        {
            profiles.push_back(SourceVertexGrowthProfile{
                vertex.source_vertex_id,
                VertexGrowthProfile{
                    command_options.first_height,
                    command_options.growth_ratio,
                    command_options.layer_count}});
        }

        RegularLayerGrowthOptions growth_options;
        growth_options.cell_quality.maximum_skewness =
            command_options.maximum_skewness;
        growth_options.max_neighbor_layer_difference =
            command_options.max_neighbor_layer_difference;
        growth_options.isotropic_height =
            command_options.isotropic_height;
        const auto growth = generateRegularLayers(
            surface.value(),
            topology.value(),
            patch.value(),
            front.value(),
            profiles,
            growth_options);
        if (!growth.hasValue())
        {
            error << "failed to generate boundary layers\n";
            return 6;
        }

        const auto parent = command_options.output_prefix.parent_path();
        std::error_code directory_error;
        if (!parent.empty())
        {
            std::filesystem::create_directories(
                parent,
                directory_error);
        }
        if (directory_error)
        {
            error << "failed to create output directory\n";
            return 7;
        }

        const auto volume_path = std::filesystem::path(
            command_options.output_prefix.string() +
            "_boundary_layer.vtk");
        const auto farfield_path = std::filesystem::path(
            command_options.output_prefix.string() +
            "_farfield_boundary.vtk");
        const auto volume_status = writeLegacyVtk(
            volume_path,
            growth.value().mesh);
        const auto farfield_status = writeLegacyVtk(
            farfield_path,
            growth.value().farfield_boundary);
        if (!volume_status.hasValue() || !farfield_status.hasValue())
        {
            error << "failed to write VTK output\n";
            return 7;
        }

        output << "input_vertices=" << surface.value().vertices.size()
               << '\n'
               << "input_faces=" << surface.value().faces.size()
               << '\n'
               << "volume_cells=" << growth.value().mesh.cells.size()
               << '\n'
               << "farfield_faces="
               << growth.value().farfield_boundary.faces.size()
               << '\n'
               << "maximum_skewness="
               << command_options.maximum_skewness
               << '\n'
               << "max_neighbor_layer_difference="
               << command_options.max_neighbor_layer_difference
               << '\n'
               << "isotropic_height="
               << command_options.isotropic_height
               << '\n';
        printStopReasonCounts(output, growth.value());
        return 0;
    }
}
