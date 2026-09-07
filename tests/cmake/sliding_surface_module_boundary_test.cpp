#include <boundary_mesh/growth/sliding_surface.hpp>

int main()
{
    boundary_mesh::SlidingSurfaceSet surfaces;
    return surfaces.surfaces().empty() ? 0 : 1;
}
