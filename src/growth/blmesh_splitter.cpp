#include <cstddef>
#include <set>
#include <vector>

#include <boundary_mesh/growth/detail/blmesh_splitter.hpp>

namespace boundary_mesh::detail
{
    std::vector<std::set<std::size_t>>
    BlmeshSplitter::combinations(const std::set<std::size_t> &input)
    {
        const std::size_t input_size = input.size();
        if (combinations_by_size_.find(input_size) ==
            combinations_by_size_.end())
        {
            for (std::size_t size = 2; size <= input_size; ++size)
            {
                if (!combinations_by_size_[size].empty()) continue;
                if (size == 2)
                {
                    combinations_by_size_[size].push_back({0, 1});
                    combinations_by_size_[size].push_back({0});
                    combinations_by_size_[size].push_back({1});
                    combinations_by_size_[size].push_back({});
                }
                else
                {
                    combinations_by_size_[size] =
                        combinations_by_size_[size - 1];
                    const std::size_t previous_count =
                        combinations_by_size_[size - 1].size();
                    for (std::size_t index = 0;
                         index < previous_count;
                         ++index)
                    {
                        std::vector<std::size_t> candidate =
                            combinations_by_size_[size - 1][index];
                        candidate.push_back(size - 1);
                        combinations_by_size_[size].push_back(
                            std::move(candidate));
                    }
                }
            }
        }

        std::vector<std::set<std::size_t>> result;
        std::vector<std::size_t> indices;
        indices.reserve(input.size());
        for (const std::size_t value : input) indices.push_back(value);

        for (const std::vector<std::size_t> &combination :
             combinations_by_size_[input_size])
        {
            if (combination.size() < 2) continue;
            std::set<std::size_t> selected;
            for (const std::size_t index : combination)
            {
                selected.insert(indices[index]);
            }
            result.push_back(std::move(selected));
        }
        return result;
    }
}
