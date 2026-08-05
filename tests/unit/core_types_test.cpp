#include <cstdint>
#include <type_traits>

#include <boundary_mesh/core/types.hpp>

int main()
{
    using namespace boundary_mesh;

    static_assert(std::is_same_v<Scalar, double>);
    static_assert(std::is_same_v<VertexId, std::uint32_t>);
    static_assert(std::is_same_v<EdgeId, std::uint32_t>);
    static_assert(std::is_same_v<SurfaceFaceId, std::uint32_t>);
    static_assert(std::is_same_v<VolumeCellId, std::uint32_t>);

    const Point3 point{1.0, 2.0, 3.0};
    const Vector3 direction{0.0, 1.0, 0.0};

    if (point.x() != 1.0 || point.y() != 2.0 || point.z() != 3.0)
    {
        return 1;
    }

    if (direction.norm() != 1.0)
    {
        return 2;
    }

    return 0;
}
