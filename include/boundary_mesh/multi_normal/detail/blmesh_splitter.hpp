#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <vector>

namespace boundary_mesh::detail
{
    class BlmeshSplitter
    {
    public:
        std::vector<std::set<std::size_t>>
        combinations(const std::set<std::size_t> &input);

    private:
        std::map<std::size_t, std::vector<std::vector<std::size_t>>>
            combinations_by_size_;
    };
}
