#include <iostream>
#include <string>
#include <vector>

#include <cli/boundary_mesh_command.hpp>

int main(int argc, char **argv)
{
    std::vector<std::string> arguments;
    arguments.reserve(
        argc > 1 ? static_cast<std::size_t>(argc - 1) : 0);
    for (int index = 1; index < argc; ++index)
    {
        arguments.emplace_back(argv[index]);
    }
    return boundary_mesh::runBoundaryMeshCommand(
        arguments,
        std::cout,
        std::cerr);
}
