#include <cmath>
#include <vector>

#include <boundary_mesh/growth/detail/blmesh_geometry.hpp>

namespace
{
    using namespace boundary_mesh;

    bool near(const Vector3 &actual, const Vector3 &expected)
    {
        return (actual - expected).norm() <= Scalar{1e-12};
    }
}

int main()
{
    using namespace boundary_mesh;
    using namespace boundary_mesh::detail;

    const Vector3 z = Vector3::UnitZ();
    if (!near(blmeshMostNormal({z}), z)) return 1;

    const Vector3 sixty{
        std::sqrt(Scalar{3}) / Scalar{2}, Scalar{0}, Scalar{0.5}};
    const Vector3 two_expected{Scalar{0.5}, Scalar{0},
                               std::sqrt(Scalar{3}) / Scalar{2}};
    if (!near(blmeshMostNormal({z, sixty}), two_expected)) return 2;

    const Vector3 equal = Vector3::Ones().normalized();
    if (!near(blmeshMostNormal(
                  {Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ()}),
              equal))
    {
        return 3;
    }
    if (std::abs(blmeshMinCos(equal,
            {Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ()}) -
            Scalar{1} / std::sqrt(Scalar{3})) > Scalar{1e-12})
    {
        return 4;
    }
    return 0;
}
