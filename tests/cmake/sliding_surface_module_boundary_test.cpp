#include <vector>

#include <boundary_mesh/sliding/sliding_constraint_builder.hpp>
#include <boundary_mesh/sliding/sliding_surface_builder.hpp>

int main()
{
    boundary_mesh::SlidingSurfaceSet surfaces;
    const std::vector<boundary_mesh::SlidingVertexInput> vertices;
    const auto result = boundary_mesh::SlidingConstraintBuilder{}.build(
        surfaces, vertices, 1.0, 1.0e-12, 0);
    return result.hasValue() ? 0 : 1;
}
