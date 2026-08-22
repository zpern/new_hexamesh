#pragma once

#include <filesystem>
#include <string>

namespace boundary_mesh::test
{
    enum class FixtureZoneKind
    {
        Unstructured,
        Structured
    };

    enum class FixtureElementKind
    {
        TriangleAndQuad,
        Line,
        InvalidTriangleReference
    };

    struct SingleZoneFixtureOptions
    {
        int cell_dimension{2};
        int physical_dimension{3};
        std::string zone_name{"1"};
        FixtureZoneKind zone_kind{FixtureZoneKind::Unstructured};
        FixtureElementKind element_kind{
            FixtureElementKind::TriangleAndQuad};
        bool write_coordinate_z{true};
        bool write_non_finite_coordinate{false};
    };

    void writeSingleZoneMixedSurface(
        const std::filesystem::path &path);

    void writeSingleZoneSurface(
        const std::filesystem::path &path,
        const SingleZoneFixtureOptions &options);

    void writeEmptyCgns(
        const std::filesystem::path &path);

    void writeTwoBaseCgns(
        const std::filesystem::path &path);
}
