#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include <boundary_mesh/growth/growth_profile_builder.hpp>

namespace boundary_mesh
{
    const VertexGrowthProfile *GrowthProfileTable::find(
        VertexId source_vertex_id) const noexcept
    {
        const auto found = std::lower_bound(
            entries_.begin(),
            entries_.end(),
            source_vertex_id,
            [](const SourceVertexGrowthProfile &entry, VertexId id)
            {
                return entry.source_vertex_id < id;
            });

        if (found == entries_.end() ||
            found->source_vertex_id != source_vertex_id)
        {
            return nullptr;
        }
        return &found->profile;
    }

    Result<Scalar, NonFiniteLayerHeight>
    GrowthProfileTable::height(
        VertexId source_vertex_id,
        std::uint32_t layer) const
    {
        using HeightResult = Result<Scalar, NonFiniteLayerHeight>;

        const VertexGrowthProfile *profile = find(source_vertex_id);
        if (profile == nullptr || layer == 0)
        {
            return HeightResult::failure(
                NonFiniteLayerHeight{source_vertex_id, layer});
        }

        Scalar value = profile->first_height;
        for (std::uint32_t current = 1; current < layer; ++current)
        {
            value *= profile->growth_ratio;
            if (!std::isfinite(value))
            {
                return HeightResult::failure(
                    NonFiniteLayerHeight{source_vertex_id, layer});
            }
        }

        return HeightResult::success(value);
    }

    Result<GrowthProfileTable, GrowthProfileError>
    GrowthProfileBuilder::build(
        const GrowthPatch &patch,
        const std::vector<SourceVertexGrowthProfile> &profiles) const
    {
        using BuildResult =
            Result<GrowthProfileTable, GrowthProfileError>;

        std::vector<SourceVertexGrowthProfile> sorted = profiles;
        std::sort(
            sorted.begin(),
            sorted.end(),
            [](const SourceVertexGrowthProfile &first,
               const SourceVertexGrowthProfile &second)
            {
                return first.source_vertex_id < second.source_vertex_id;
            });

        for (std::size_t index = 1; index < sorted.size(); ++index)
        {
            if (sorted[index - 1].source_vertex_id ==
                sorted[index].source_vertex_id)
            {
                return BuildResult::failure(
                    DuplicateVertexGrowthProfile{
                        sorted[index].source_vertex_id});
            }
        }

        for (const SourceVertexGrowthProfile &entry : sorted)
        {
            const auto patch_vertex = std::lower_bound(
                patch.vertices().begin(),
                patch.vertices().end(),
                entry.source_vertex_id,
                [](const PatchVertex &vertex, VertexId id)
                {
                    return vertex.source_vertex_id < id;
                });
            if (patch_vertex == patch.vertices().end() ||
                patch_vertex->source_vertex_id != entry.source_vertex_id)
            {
                return BuildResult::failure(
                    UnknownVertexGrowthProfile{entry.source_vertex_id});
            }

            if (!std::isfinite(entry.profile.first_height) ||
                entry.profile.first_height <= Scalar{0})
            {
                return BuildResult::failure(
                    InvalidFirstHeight{
                        entry.source_vertex_id,
                        entry.profile.first_height});
            }
            if (!std::isfinite(entry.profile.growth_ratio) ||
                entry.profile.growth_ratio <= Scalar{0})
            {
                return BuildResult::failure(
                    InvalidGrowthRatio{
                        entry.source_vertex_id,
                        entry.profile.growth_ratio});
            }
        }

        std::vector<SourceVertexGrowthProfile> ordered;
        ordered.reserve(patch.vertices().size());
        for (const PatchVertex &vertex : patch.vertices())
        {
            const auto found = std::lower_bound(
                sorted.begin(),
                sorted.end(),
                vertex.source_vertex_id,
                [](const SourceVertexGrowthProfile &entry, VertexId id)
                {
                    return entry.source_vertex_id < id;
                });
            if (found == sorted.end() ||
                found->source_vertex_id != vertex.source_vertex_id)
            {
                return BuildResult::failure(
                    MissingVertexGrowthProfile{vertex.source_vertex_id});
            }
            ordered.push_back(*found);
        }

        return BuildResult::success(
            GrowthProfileTable{std::move(ordered)});
    }
}
