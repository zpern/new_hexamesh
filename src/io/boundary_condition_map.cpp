#include <io/boundary_condition_map.hpp>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace boundary_mesh
{
    namespace
    {
        struct ParsedEntry
        {
            BoundaryZoneEntry entry;
            std::size_t line{};
        };

        bool asciiSpace(char value)
        {
            return value == ' ' || value == '\t' ||
                   value == '\r' || value == '\n' ||
                   value == '\f' || value == '\v';
        }

        std::string trimAscii(const std::string &value)
        {
            const auto first = std::find_if_not(
                value.begin(), value.end(), asciiSpace);
            const auto last = std::find_if_not(
                value.rbegin(), value.rend(), asciiSpace).base();
            return first >= last ? std::string{} :
                                   std::string(first, last);
        }

        Result<std::uint32_t, BoundaryMapError> parseZoneId(
            const std::filesystem::path &path,
            const std::string &text,
            std::size_t line)
        {
            std::uint32_t value{};
            const char *begin = text.data();
            const char *end = begin + text.size();
            const auto parsed = std::from_chars(begin, end, value, 10);
            if (text.empty() || parsed.ec != std::errc{} ||
                parsed.ptr != end || value == 0)
            {
                return Result<std::uint32_t, BoundaryMapError>::failure(
                    {BoundaryMapErrorCode::InvalidZoneId, path, line, 0});
            }
            return Result<std::uint32_t, BoundaryMapError>::success(value);
        }

        std::optional<SurfaceBoundaryKind> sectionKind(
            const std::string &name)
        {
            if (name == "Far:") return SurfaceBoundaryKind::Farfield;
            if (name == "Wall:") return SurfaceBoundaryKind::Wall;
            if (name == "Symmetry:") return SurfaceBoundaryKind::Symmetry;
            if (name == "Internal:") return SurfaceBoundaryKind::Internal;
            return std::nullopt;
        }
    }

    const BoundaryZoneEntry *BoundaryZoneMap::find(
        std::uint32_t zone_id) const noexcept
    {
        const auto found = std::lower_bound(
            entries.begin(),
            entries.end(),
            zone_id,
            [](const BoundaryZoneEntry &entry, std::uint32_t id)
            {
                return entry.zone_id < id;
            });
        return found != entries.end() && found->zone_id == zone_id
            ? &*found
            : nullptr;
    }

    BoundaryMapResult readBoundaryConditionMap(
        const std::filesystem::path &path,
        const std::vector<std::uint32_t> &available_zone_ids)
    {
        std::ifstream input(path);
        if (!input.is_open())
        {
            return BoundaryMapResult::failure(
                {BoundaryMapErrorCode::FileOpenFailure, path, 0, 0});
        }

        bool has_section = false;
        SurfaceBoundaryKind current_kind =
            SurfaceBoundaryKind::Farfield;
        std::vector<ParsedEntry> parsed_entries;
        std::string raw_line;
        std::size_t line_number = 0;
        while (std::getline(input, raw_line))
        {
            ++line_number;
            const std::string line = trimAscii(raw_line);
            if (line.empty()) continue;
            if (const auto kind = sectionKind(line))
            {
                has_section = true;
                current_kind = *kind;
                continue;
            }
            if (line.find(':') != std::string::npos)
            {
                return BoundaryMapResult::failure(
                    {BoundaryMapErrorCode::InvalidSection,
                     path,
                     line_number,
                     0});
            }

            const auto zone = parseZoneId(path, line, line_number);
            if (!zone.hasValue())
            {
                return BoundaryMapResult::failure(zone.error());
            }
            if (!has_section)
            {
                return BoundaryMapResult::failure(
                    {BoundaryMapErrorCode::ZoneOutsideSection,
                     path,
                     line_number,
                     zone.value()});
            }
            const auto duplicate = std::find_if(
                parsed_entries.begin(),
                parsed_entries.end(),
                [&](const ParsedEntry &entry)
                {
                    return entry.entry.zone_id == zone.value();
                });
            if (duplicate != parsed_entries.end())
            {
                return BoundaryMapResult::failure(
                    {BoundaryMapErrorCode::DuplicateZone,
                     path,
                     line_number,
                     zone.value()});
            }
            parsed_entries.push_back(
                {{zone.value(), current_kind, zone.value()}, line_number});
        }

        std::vector<std::uint32_t> available = available_zone_ids;
        std::sort(available.begin(), available.end());
        available.erase(
            std::unique(available.begin(), available.end()),
            available.end());
        std::sort(
            parsed_entries.begin(),
            parsed_entries.end(),
            [](const ParsedEntry &left, const ParsedEntry &right)
            {
                return left.entry.zone_id < right.entry.zone_id;
            });

        for (const ParsedEntry &parsed : parsed_entries)
        {
            if (!std::binary_search(
                    available.begin(), available.end(), parsed.entry.zone_id))
            {
                return BoundaryMapResult::failure(
                    {BoundaryMapErrorCode::UnknownZone,
                     path,
                     parsed.line,
                     parsed.entry.zone_id});
            }
        }
        for (const std::uint32_t zone_id : available)
        {
            const auto found = std::lower_bound(
                parsed_entries.begin(),
                parsed_entries.end(),
                zone_id,
                [](const ParsedEntry &entry, std::uint32_t id)
                {
                    return entry.entry.zone_id < id;
                });
            if (found == parsed_entries.end() ||
                found->entry.zone_id != zone_id)
            {
                return BoundaryMapResult::failure(
                    {BoundaryMapErrorCode::MissingZone, path, 0, zone_id});
            }
        }

        BoundaryZoneMap output;
        output.entries.reserve(parsed_entries.size());
        for (ParsedEntry &parsed : parsed_entries)
        {
            output.entries.push_back(std::move(parsed.entry));
        }
        return BoundaryMapResult::success(std::move(output));
    }
}
