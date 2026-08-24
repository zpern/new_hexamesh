#include <cassert>
#include <cstdint>
#include <limits>
#include <vector>

#include <boundary_mesh/transition/reserved_layer_growth.hpp>

using namespace boundary_mesh;

int main()
{
    assert(regularLayerCount(0) == 0);
    assert(regularLayerCount(1) == 0);
    assert(regularLayerCount(2) == 0);
    assert(regularLayerCount(7) == 5);
    assert(occupiedLayerCount(0) == 0);
    assert(occupiedLayerCount(1) == 0);
    assert(occupiedLayerCount(2) == 1);
    assert(occupiedLayerCount(7) == 6);

    const std::vector<SourceVertexGrowthProfile> requested{
        {VertexId{4}, VertexGrowthProfile{0.1, 1.2, 20}}};
    const auto trial = makeReservedTrialProfiles(requested);
    assert(trial.hasValue());
    assert(trial.value()[0].source_vertex_id == VertexId{4});
    assert(trial.value()[0].profile.layer_count == 22);
    assert(trial.value()[0].profile.first_height == 0.1);
    assert(trial.value()[0].profile.growth_ratio == 1.2);

    const std::vector<SourceVertexGrowthProfile> overflow{
        {VertexId{9},
         VertexGrowthProfile{
             0.1,
             1.0,
             std::numeric_limits<std::uint32_t>::max()}}};
    const auto invalid = makeReservedTrialProfiles(overflow);
    assert(!invalid.hasValue());
    assert(invalid.error().source_vertex_id == VertexId{9});
}
