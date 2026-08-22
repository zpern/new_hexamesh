#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <io/boundary_condition_map.hpp>

namespace
{
    using namespace boundary_mesh;

    bool writeText(
        const std::filesystem::path &path,
        const std::string &text)
    {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output << text;
        return output.good();
    }

    bool hasError(
        const BoundaryMapResult &result,
        BoundaryMapErrorCode code,
        std::size_t line,
        std::uint32_t zone_id)
    {
        return !result.hasValue() &&
               result.error().code == code &&
               result.error().line == line &&
               result.error().zone_id == zone_id;
    }
}

int main()
{
    using namespace boundary_mesh;

    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        "boundary_mesh_boundary_map_test";
    std::filesystem::create_directories(root);
    const std::filesystem::path path = root / "case.bc.txt";

    if (!writeText(path, "Far:\n1\n2\n\nWall:\n3\n4\n")) return 1;
    const auto valid = readBoundaryConditionMap(path, {1, 2, 3, 4});
    if (!valid.hasValue() || valid.value().entries.size() != 4) return 2;
    const BoundaryZoneEntry *far = valid.value().find(1);
    const BoundaryZoneEntry *wall = valid.value().find(3);
    const BoundaryZoneEntry *wall4 = valid.value().find(4);
    if (far == nullptr || wall == nullptr || wall4 == nullptr ||
        far->kind != SurfaceBoundaryKind::Farfield ||
        wall->kind != SurfaceBoundaryKind::Wall ||
        wall4->region_id != 4)
    {
        return 3;
    }

    const auto missing_file = readBoundaryConditionMap(
        root / "missing.bc.txt", {1});
    if (!hasError(
            missing_file,
            BoundaryMapErrorCode::FileOpenFailure,
            0,
            0))
    {
        return 4;
    }
    if (!writeText(path, "Near:\n1\n") ||
        !hasError(
            readBoundaryConditionMap(path, {1}),
            BoundaryMapErrorCode::InvalidSection,
            1,
            0))
    {
        return 5;
    }
    if (!writeText(path, "1\nFar:\n") ||
        !hasError(
            readBoundaryConditionMap(path, {1}),
            BoundaryMapErrorCode::ZoneOutsideSection,
            1,
            1))
    {
        return 6;
    }
    const std::vector<std::string> invalid_ids{
        "Far:\nabc\n", "Far:\n0\n", "Far:\n4294967296\n",
        "Far:\n+1\n", "Far:\n1x\n"};
    for (const std::string &contents : invalid_ids)
    {
        if (!writeText(path, contents) ||
            !hasError(
                readBoundaryConditionMap(path, {1}),
                BoundaryMapErrorCode::InvalidZoneId,
                2,
                0))
        {
            return 7;
        }
    }
    if (!writeText(path, "Far:\n1\nWall:\n1\n") ||
        !hasError(
            readBoundaryConditionMap(path, {1}),
            BoundaryMapErrorCode::DuplicateZone,
            4,
            1))
    {
        return 8;
    }
    if (!writeText(path, "Far:\n1\n") ||
        !hasError(
            readBoundaryConditionMap(path, {1, 2}),
            BoundaryMapErrorCode::MissingZone,
            0,
            2))
    {
        return 9;
    }
    if (!writeText(path, "Far:\n1\n2\n") ||
        !hasError(
            readBoundaryConditionMap(path, {1}),
            BoundaryMapErrorCode::UnknownZone,
            3,
            2))
    {
        return 10;
    }

    std::error_code cleanup_error;
    std::filesystem::remove_all(root, cleanup_error);
}
