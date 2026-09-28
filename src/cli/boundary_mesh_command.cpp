#include <cli/boundary_mesh_command.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <locale>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>
#include <regex>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/io/cgns_surface_reader.hpp>
#include <boundary_mesh/io/legacy_vtk_writer.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/boundary_layer/boundary_layer_generator.hpp>

namespace boundary_mesh
{
    void printSurfaceTopologyError(
        std::ostream &error,
        const SurfaceTopologyError &topology_error)
    {
        std::visit(
            [&error](const auto &detail)
            {
                using Error = std::decay_t<decltype(detail)>;
                if constexpr (std::is_same_v<Error, EmptySurface>)
                    error << "empty surface";
                else if constexpr (std::is_same_v<Error, FaceTagCountMismatch>)
                    error << "face tag count mismatch: faces=" << detail.face_count
                          << " face_tags=" << detail.face_tag_count;
                else if constexpr (std::is_same_v<Error, NonFiniteVertex>)
                    error << "non-finite vertex: vertex=" << detail.vertex_id;
                else if constexpr (std::is_same_v<Error, InvalidVertexReference>)
                    error << "invalid vertex reference: face=" << detail.face_id
                          << " vertex=" << detail.vertex_id;
                else if constexpr (std::is_same_v<Error, DegenerateFace>)
                    error << "degenerate face: face=" << detail.face_id;
                else if constexpr (std::is_same_v<Error, DuplicateFace>)
                    error << "duplicate face: first_face=" << detail.first_face_id
                          << " duplicate_face=" << detail.duplicate_face_id;
                else if constexpr (std::is_same_v<Error, BoundaryEdge>)
                    error << "boundary edge: vertices=" << detail.edge_vertices[0]
                          << ',' << detail.edge_vertices[1]
                          << " face=" << detail.face_id;
                else if constexpr (std::is_same_v<Error, NonManifoldEdge>)
                    error << "non-manifold edge: vertices=" << detail.edge_vertices[0]
                          << ',' << detail.edge_vertices[1]
                          << " faces=" << detail.face_ids[0] << ','
                          << detail.face_ids[1] << ',' << detail.face_ids[2];
                else if constexpr (std::is_same_v<Error, InconsistentOrientation>)
                    error << "inconsistent orientation: vertices="
                          << detail.edge_vertices[0] << ',' << detail.edge_vertices[1]
                          << " faces=" << detail.first_face_id << ','
                          << detail.second_face_id;
            },
            topology_error);
    }

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

        bool parseBoolean(const std::string &text, bool &value)
        {
            if (text == "true" || text == "1")
            {
                value = true;
                return true;
            }
            if (text == "false" || text == "0")
            {
                value = false;
                return true;
            }
            return false;
        }

        bool loadJsonConfig(const std::filesystem::path &path,
                            BoundaryMeshCommandOptions &options,
                            std::string &message)
        {
            std::ifstream file(path);
            if (!file) { message = "failed to open config: " + path.string(); return false; }
            const std::string json((std::istreambuf_iterator<char>(file)), {});
            const auto get = [&](const char *key) {
                std::smatch match;
                const std::regex pattern(std::string("\\\"") + key + "\\\"\\s*:\\s*(?:\\\"([^\\\"]*)\\\"|([^,}\\s]+))");
                if (!std::regex_search(json, match, pattern)) return std::string{};
                return match[1].matched ? match[1].str() : match[2].str();
            };
            std::string value;
            value = get("input"); if (!value.empty()) options.input = value;
            value = get("output_prefix"); if (!value.empty()) options.output_prefix = value;
            value = get("first_height"); if (value.empty() || !parseScalar(value, options.first_height)) { message = "invalid first_height in config"; return false; }
            value = get("growth_ratio"); if (value.empty() || !parseScalar(value, options.growth_ratio)) { message = "invalid growth_ratio in config"; return false; }
            value = get("layer_count"); if (value.empty() || !parseUnsigned(value, options.layer_count)) { message = "invalid layer_count in config"; return false; }
            value = get("maximum_skewness"); if (!value.empty() && !parseScalar(value, options.maximum_skewness)) { message = "invalid maximum_skewness in config"; return false; }
            value = get("max_layer_diff"); if (!value.empty() && !parseUnsigned(value, options.max_layer_diff)) { message = "invalid max_layer_diff in config"; return false; }
            value = get("isotropic_height"); if (!value.empty() && !parseScalar(value, options.isotropic_height)) { message = "invalid isotropic_height in config"; return false; }
            value = get("multi_normal"); if (!value.empty() && !parseBoolean(value, options.multi_normal_enabled)) { message = "invalid multi_normal in config"; return false; }
            value = get("split"); if (!value.empty() && !parseBoolean(value, options.split_failed_hexa_columns)) { message = "invalid split in config"; return false; }
            value = get("debug_log"); if (!value.empty() && !parseBoolean(value, options.debug_log_enabled)) { message = "invalid debug_log in config"; return false; }
            return true;
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
            if (!arguments.empty() && arguments[0] == "--config")
            {
                if (arguments.size() < 2) { message = "missing value for --config"; return ParseStatus::Failure; }
                options.config_file = arguments[1];
                if (!loadJsonConfig(options.config_file, options, message)) return ParseStatus::Failure;
                has_input = !options.input.empty();
                has_first_height = options.first_height > Scalar{0};
                has_growth_ratio = options.growth_ratio > Scalar{0};
                has_layer_count = options.layer_count > 0;
            }

            const std::size_t start = options.config_file.empty() ? 0 : 2;
            for (std::size_t index = start;
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
                else if (name == "--max-layer-diff")
                {
                    if (!parseUnsigned(
                            value,
                            options.max_layer_diff))
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
                else if (name == "--multi-normal")
                {
                    if (!parseBoolean(
                            value,
                            options.multi_normal_enabled))
                    {
                        message = "invalid multi-normal flag";
                        return ParseStatus::Failure;
                    }
                }
                else if (name == "--debuglog")
                {
                    if (!parseBoolean(value, options.debug_log_enabled))
                    {
                        message = "invalid debuglog flag";
                        return ParseStatus::Failure;
                    }
                }
                else if (name == "--split")
                {
                    if (!parseBoolean(value, options.split_failed_hexa_columns))
                    {
                        message = "invalid split flag";
                        return ParseStatus::Failure;
                    }
                }
                else if (name == "--verify-transition-rebuilds")
                {
                    if (!parseBoolean(value, options.verify_transition_rebuilds))
                    {
                        message = "invalid transition verification flag";
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
                << "[--multi-normal true|false] "
                << "[--split true|false] "
                << "[--verify-transition-rebuilds true|false] "
                << "[--debuglog true|false] "
                << "[--output-prefix PATH]\n";
            error << "   or: boundary_mesh_cli --config FILE [command-line overrides]\n";
        }

        std::vector<std::uint32_t> regionIds(
            const SurfaceMesh &mesh,
            SurfaceBoundaryKind kind)
        {
            std::vector<std::uint32_t> ids;
            for (std::size_t index = 0; index < mesh.faces.size(); ++index)
            {
                if (index < mesh.face_tags.size() &&
                    mesh.face_tags[index].kind == kind)
                    ids.push_back(mesh.face_tags[index].region_id);
            }
            std::sort(ids.begin(), ids.end());
            ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
            return ids;
        }

        void printRegionLine(
            std::ostream &output,
            const char *name,
            const std::vector<std::uint32_t> &ids)
        {
            if (ids.empty()) return;
            output << name << '=';
            for (std::size_t i = 0; i < ids.size(); ++i)
                output << (i == 0 ? "" : ",") << ids[i];
            output << '\n';
        }

        void printInputBoundaries(std::ostream &output, const SurfaceMesh &mesh)
        {
            printRegionLine(output, "symmetry_regions",
                            regionIds(mesh, SurfaceBoundaryKind::Symmetry));
            printRegionLine(output, "internal_regions",
                            regionIds(mesh, SurfaceBoundaryKind::Internal));
            printRegionLine(output, "wall_regions",
                            regionIds(mesh, SurfaceBoundaryKind::Wall));
            printRegionLine(output, "far_regions",
                            regionIds(mesh, SurfaceBoundaryKind::Farfield));
            const auto wall_count = std::count_if(
                mesh.face_tags.begin(), mesh.face_tags.end(),
                [](const SurfaceBoundaryTag &tag)
                { return tag.kind == SurfaceBoundaryKind::Wall; });
            output << "wall_faces=" << wall_count << '\n';
        }

        const char *stopReasonName(FaceStopReason reason)
        {
            switch (reason)
            {
            case FaceStopReason::None: return "None";
            case FaceStopReason::VertexLayerLimit: return "VertexLayerLimit";
            case FaceStopReason::DegenerateCandidate: return "DegenerateCandidate";
            case FaceStopReason::ReversedCandidate: return "ReversedCandidate";
            case FaceStopReason::LocallyInvertedCandidate: return "LocallyInvertedCandidate";
            case FaceStopReason::SkewnessExceeded: return "SkewnessExceeded";
            case FaceStopReason::Collision: return "Collision";
            case FaceStopReason::SlidingProjectionFailure: return "SlidingProjectionFailure";
            case FaceStopReason::NeighborLayerConstraint: return "NeighborLayerConstraint";
            case FaceStopReason::IsotropicHeightReached: return "IsotropicHeightReached";
            }
            return "None";
        }

        bool writeDebugLog(
            const std::filesystem::path &path,
            const RegularLayerGrowthResult &growth)
        {
            std::vector<FaceGrowthRecord> faces = growth.faces;
            std::sort(faces.begin(), faces.end(),
                      [](const FaceGrowthRecord &a, const FaceGrowthRecord &b)
                      { return a.source_face_id < b.source_face_id; });
            std::ofstream debug(path);
            if (!debug) return false;
            for (const FaceGrowthRecord &face : faces)
                debug << face.source_face_id << ' '
                      << stopReasonName(face.stop_reason)
                      << " accepted_layers=" << face.accepted_layer_count
                      << " stop_layer=" << face.stop_layer << '\n';
            for (const auto &diagnostic :
                 growth.terminal_transition_diagnostics)
                debug << (diagnostic.split_succeeded ? "warning" : "error")
                      << " terminal_quad_transition source_face="
                      << diagnostic.source_face_id
                      << " layer=" << diagnostic.layer
                      << " cell=" << diagnostic.cell_id
                      << " aspect_ratio=" << diagnostic.aspect_ratio
                      << " reason=" << diagnostic.reason << '\n';
            return static_cast<bool>(debug);
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
        printInputBoundaries(output, surface.value());

        const auto topology = SurfaceTopologyBuilder{}.build(surface.value());
        if (!topology.hasValue())
        {
            error << "failed to build surface topology: ";
            printSurfaceTopologyError(error, topology.error());
            if (const auto *detail = std::get_if<InconsistentOrientation>(
                    &topology.error()))
            {
                error << " regions="
                      << surface.value().face_tags[detail->first_face_id].region_id
                      << ','
                      << surface.value().face_tags[detail->second_face_id].region_id;
            }
            error << '\n';
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
        growth_options.cell_quality.maximum_skewness = command_options.maximum_skewness;
        growth_options.max_layer_diff = command_options.max_layer_diff;
        growth_options.isotropic_height = command_options.isotropic_height;
        growth_options.verify_transition_rebuilds = command_options.verify_transition_rebuilds;
        growth_options.split_failed_hexa_columns =
            command_options.split_failed_hexa_columns;

        MultiNormalOptions multi_normal_options;
        multi_normal_options.enabled = command_options.multi_normal_enabled;
        multi_normal_options.transition_height = command_options.first_height;
        if (command_options.multi_normal_enabled)
            output << "Generating multi-normal boundary layer\n";

        const auto growth = generateBoundaryLayers(
            surface.value(),
            topology.value(),
            patch.value(),
            front.value(),
            profiles,
            multi_normal_options,
            growth_options);

        if (!growth.hasValue())
        {
            error << "failed to generate boundary layers"
                  << " (category=" << growth.error().index();
            if (const auto *regular =
                    std::get_if<RegularLayerGenerationFailure>(
                        &growth.error()))
            {
                error << ", cause=" << regular->cause.index();
                if (const auto *boundary =
                        std::get_if<TransitionBoundaryError>(
                            &regular->cause))
                {
                    if (const auto *unresolved =
                            std::get_if<UnresolvedTransitionCollision>(
                                boundary))
                    {
                        error << ", unresolved_transition_owners="
                              << unresolved->owners.size();
                        if (!unresolved->owners.empty())
                            error << ", first_source_face="
                                  << unresolved->owners.front().source_face_id
                                  << ", first_layer="
                                  << unresolved->owners.front().layer
                                  << ", first_role="
                                  << static_cast<int>(
                                      unresolved->owners.front().role);
                    }
                }
                if (const auto *transition =
                        std::get_if<TransitionTemplateError>(
                            &regular->cause))
                {
                    error << ", transition=" << transition->index();
                    if (const auto *invalid =
                            std::get_if<InvalidTransitionTemplateInput>(
                                transition))
                        error << ", source_face="
                              << invalid->source_face_id
                              << ", stage=" << invalid->stage;
                }
            }
            error << ")\n";
            return 6;
        }

        for (const auto &diagnostic :
             growth.value().regular.terminal_transition_diagnostics)
            error << (diagnostic.split_succeeded
                          ? "warning: terminal quad transition failed; column split succeeded"
                          : "error: terminal quad transition failed; column split failed")
                  << " source_face=" << diagnostic.source_face_id
                  << " layer=" << diagnostic.layer
                  << " cell=" << diagnostic.cell_id
                  << " reason=" << diagnostic.reason << '\n';

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
        const auto top_path = std::filesystem::path(
            command_options.output_prefix.string() +
            "_boundary_layer_top.vtk");

        const auto write_started = std::chrono::steady_clock::now();
        const auto volume_status = writeLegacyVtk(
            volume_path,
            growth.value().mesh);
        const auto farfield_status = writeLegacyVtk(
            farfield_path,
            growth.value().farfield_boundary);
        const auto top_status = writeLegacyVtk(
            top_path,
            growth.value().top_surface);
        output << "temporary vtk-output ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - write_started).count() << '\n';
        if (!volume_status.hasValue() ||
            !farfield_status.hasValue() ||
            !top_status.hasValue())
        {
            const auto report = [&](const char *label,
                                    const VtkWriteStatus &status)
            {
                if (status.hasValue()) return;
                const auto &detail = status.error();
                error << "failed to write " << label << " VTK output: path="
                      << detail.path.string() << " code="
                      << static_cast<int>(detail.code)
                      << " cell_index=" << detail.cell_index
                      << " vertex_id=" << detail.vertex_id << '\n';
            };
            report("volume", volume_status);
            report("farfield", farfield_status);
            report("top", top_status);
            return 7;
        }

        if (command_options.debug_log_enabled)
        {
            const auto debug_path = std::filesystem::path(
                command_options.output_prefix.string() + "_debug.txt");
            if (!writeDebugLog(debug_path, growth.value().regular))
            {
                error << "failed to write debug log\n";
                return 7;
            }
        }
        return 0;
    }
}
