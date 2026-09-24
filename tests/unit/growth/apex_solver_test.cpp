#include <cmath>
#include <iomanip>
#include <iostream>

#include <boundary_mesh/growth/apex_solver.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

using namespace boundary_mesh;

int main()
{
    const std::array<Point3, 4> planar{{
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}}};
    const Point3 target{0.5,0.5,0.25};
    const auto direct = solvePyramidApex({planar,target});
    if (direct.status != ApexSolverStatus::Valid || !direct.apex ||
        (*direct.apex-target).norm() > 1e-10)
        return 1;

    const std::array<Point3, 4> warped{{
        {0,0,0}, {1,0,0.2}, {1,1,-0.5}, {0,1,0.2}}};
    const auto projected = solvePyramidApex({warped,{0.5,0.5,0.01}});
    if (projected.status != ApexSolverStatus::Valid || !projected.apex ||
        !projected.region.contains(*projected.apex) ||
        (*projected.apex-Point3{0.5,0.5,0.01}).norm() < 1e-4)
        return 2;
    const auto quality = evaluatePyramid({{
        warped[0],warped[1],warped[2],warped[3],*projected.apex}});
    if (!quality.hasValue() ||
        quality.value().validity != VolumeCellValidity::Valid ||
        quality.value().minimum_local_jacobian <= 0 ||
        quality.value().minimum_subtet_signed_volume <= 0)
        return 3;

    auto invalid = solvePyramidApex({planar,target,0.5,0.1,1});
    if (invalid.status != ApexSolverStatus::InvalidInput)
        return 4;
    const auto below = solvePyramidApex({planar,{0.5,0.5,-1}});
    if (below.status != ApexSolverStatus::Valid || !below.apex ||
        std::abs(below.apex->x()-0.5) > 1e-9 ||
        std::abs(below.apex->y()-0.5) > 1e-9 ||
        std::abs(below.apex->z()-1e-5) > 1e-9)
        return 6;
    const auto plane_sides = projectPyramidApexToPlaneSides(
        {planar,target},target,
        {{{0.5,0,0},{0.5,1,0},{0.5,0,1}}},2);
    if (plane_sides.size() != 2 ||
        !(plane_sides[0].x() > 0.5 && plane_sides[1].x() < 0.5))
        return 9;
    const std::array<Point3,4> steep{{
        {0,0,0},{1,0,0},{1,1,1},{0,1,0}}};
    const auto bounded = solvePyramidApex({
        steep,{0.5,0.5,0.1},0.001,0.1,0.001});
    if (bounded.status != ApexSolverStatus::EmptyFeasibleRegion)
        return 8;

    // Source face 42722, layer 8: top quad of Hexa cell 385326 in the
    // anisotropic 9-layer diagnostic VTK output.
    const std::array<Point3,4> face42722{{
        {210.5980435801371,65.135212754218472,-13.921674563524869},
        {210.45596410544081,65.096078157422411,-13.876560299917257},
        {207.12340324351837,65.094460971201983,-13.95448757283272},
        {207.35010379647761,65.132318501066763,-14.000235086547402}}};
    Point3 center = Point3::Zero();
    Scalar edge_length = 0;
    for (std::size_t i = 0; i < 4; ++i)
    {
        center += face42722[i];
        edge_length += (face42722[(i+1)%4]-face42722[i]).norm();
    }
    center /= 4;
    edge_length /= 4;
    Vector3 normal = (face42722[1]-face42722[0]).cross(face42722[2]-face42722[0]) +
        (face42722[2]-face42722[0]).cross(face42722[3]-face42722[0]);
    normal.normalize();
    const auto real = solvePyramidApex({face42722,center+0.25*edge_length*normal});
    std::cout << std::setprecision(17)
              << "42722 status=" << static_cast<int>(real.status)
              << " constraints=" << real.region.constraints.size();
    if (real.apex) std::cout << " apex=" << real.apex->transpose()
                              << " slack=" << real.minimum_slack;
    std::cout << '\n';
    if (real.status != ApexSolverStatus::Valid || !real.apex) return 5;
    const auto real_quality = evaluatePyramid({{
        face42722[0],face42722[1],face42722[2],face42722[3],*real.apex}});
    if (!real_quality.hasValue() ||
        real_quality.value().validity != VolumeCellValidity::Valid ||
        real_quality.value().minimum_local_jacobian <= 0 ||
        real_quality.value().minimum_subtet_signed_volume <= 0)
        return 7;
    return 0;
}
