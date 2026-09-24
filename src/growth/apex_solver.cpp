#include <boundary_mesh/growth/apex_solver.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/LU>

namespace boundary_mesh
{
    bool ApexFeasibleRegion::contains(
        const Point3 &point, Scalar tolerance) const
    {
        return point.allFinite() && minimumSlack(point) >= -tolerance;
    }

    Scalar ApexFeasibleRegion::minimumSlack(const Point3 &point) const
    {
        Scalar minimum = std::numeric_limits<Scalar>::infinity();
        for (const auto &constraint : constraints)
            minimum = std::min(minimum,
                constraint.normal.dot(point) - constraint.offset);
        return minimum;
    }

    namespace
    {
        bool addConstraint(ApexFeasibleRegion &region,
            const Vector3 &normal, const Point3 &origin, Scalar height)
        {
            const Scalar length = normal.norm();
            if (!std::isfinite(length) || length <= Scalar{1e-14})
                return false;
            const Vector3 unit = normal / length;
            region.constraints.push_back({unit,unit.dot(origin)+height});
            return true;
        }

        bool validQuality(const std::array<Point3, 4> &quad,
            const Point3 &apex)
        {
            const auto evaluation = evaluatePyramid({{
                quad[0],quad[1],quad[2],quad[3],apex}});
            return evaluation.hasValue() &&
                evaluation.value().validity == VolumeCellValidity::Valid &&
                evaluation.value().minimum_subtet_signed_volume > 0 &&
                evaluation.value().minimum_local_jacobian > 0;
        }
    }

    ApexSolverResult solvePyramidApex(const ApexSolverInput &input)
    {
        ApexSolverResult output;
        if (!input.target.allFinite() ||
            !std::isfinite(input.minimum_height_ratio) ||
            !std::isfinite(input.maximum_height_ratio) ||
            !std::isfinite(input.lateral_radius_ratio) ||
            input.minimum_height_ratio <= 0 ||
            input.maximum_height_ratio <= input.minimum_height_ratio ||
            input.lateral_radius_ratio <= 0)
            return output;
        Point3 center = Point3::Zero();
        Scalar length = 0;
        for (std::size_t i = 0; i < 4; ++i)
        {
            if (!input.quad[i].allFinite()) return output;
            center += input.quad[i];
            length += (input.quad[(i+1)%4]-input.quad[i]).norm();
        }
        center /= 4;
        length /= 4;
        if (!std::isfinite(length) || length <= 1e-14) return output;
        const Scalar minimum_height = length * input.minimum_height_ratio;
        const auto &q = input.quad;
        Vector3 direction = (q[1]-q[0]).cross(q[2]-q[0]) +
            (q[2]-q[0]).cross(q[3]-q[0]);
        if (direction.norm() <= length*length*1e-12) return output;
        direction.normalize();
        Vector3 tangent = Vector3::Zero();
        for (std::size_t i = 0; i < 4; ++i)
        {
            const Vector3 edge = q[(i+1)%4]-q[i];
            const Vector3 projected = edge-edge.dot(direction)*direction;
            if (projected.squaredNorm() > tangent.squaredNorm())
                tangent = projected;
        }
        if (tangent.norm() <= length*1e-12) return output;
        tangent.normalize();
        const Vector3 bitangent = direction.cross(tangent).normalized();

        auto &region = output.region;
        if (!addConstraint(region,(q[1]-q[0]).cross(q[2]-q[0]),q[0],minimum_height) ||
            !addConstraint(region,(q[2]-q[0]).cross(q[3]-q[0]),q[0],minimum_height))
            return output;
        for (const Scalar r : {Scalar{0},Scalar{0.5},Scalar{1}})
            for (const Scalar s : {Scalar{0},Scalar{0.5},Scalar{1}})
            {
                const Vector3 dr = (1-s)*(q[1]-q[0])+s*(q[2]-q[3]);
                const Vector3 ds = (1-r)*(q[3]-q[0])+r*(q[2]-q[1]);
                const Point3 base = (1-r)*(1-s)*q[0]+r*(1-s)*q[1]+r*s*q[2]+(1-r)*s*q[3];
                if (!addConstraint(region,dr.cross(ds),base,minimum_height))
                    return output;
            }
        const Scalar maximum_height = length*input.maximum_height_ratio;
        const Scalar radius = length*input.lateral_radius_ratio;
        addConstraint(region,direction,center,minimum_height);
        addConstraint(region,-direction,center,-maximum_height);
        for (const Vector3 axis : {tangent,bitangent})
        {
            addConstraint(region,axis,center,-radius);
            addConstraint(region,-axis,center,-radius);
        }
        region.constraints.insert(region.constraints.end(),
            input.additional_constraints.begin(),
            input.additional_constraints.end());

        // Solve in local length units to keep the KKT systems conditioned.
        const Point3 target = (input.target-center)/length;
        const std::size_t count = region.constraints.size();
        std::vector<Scalar> bounds(count);
        for (std::size_t i = 0; i < count; ++i)
            bounds[i] = (region.constraints[i].offset-
                region.constraints[i].normal.dot(center))/length;
        const auto feasible = [&](const Point3 &point)
        {
            for (std::size_t i = 0; i < count; ++i)
                if (region.constraints[i].normal.dot(point) < bounds[i]-1e-10)
                    return false;
            return true;
        };
        std::optional<Point3> best;
        Scalar best_distance = std::numeric_limits<Scalar>::infinity();
        if (feasible(target))
        {
            best = target;
            best_distance = 0;
        }
        const auto considerActiveSet = [&](const std::array<std::size_t,3> &active,
                                            std::size_t rank)
        {
            Eigen::Matrix<Scalar,3,3> gram =
                Eigen::Matrix<Scalar,3,3>::Zero();
            Eigen::Matrix<Scalar,3,1> rhs =
                Eigen::Matrix<Scalar,3,1>::Zero();
            for (std::size_t i = 0; i < rank; ++i)
            {
                const auto &normal_i = region.constraints[active[i]].normal;
                rhs[i] = bounds[active[i]]-normal_i.dot(target);
                for (std::size_t j = 0; j < rank; ++j)
                    gram(i,j) = normal_i.dot(
                        region.constraints[active[j]].normal);
            }
            const auto small = gram.topLeftCorner(rank,rank).eval();
            Eigen::FullPivLU<
                Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>> lu(small);
            if (lu.rank() != static_cast<Eigen::Index>(rank)) return;
            const Eigen::VectorXd multipliers = lu.solve(rhs.head(rank));
            if (!multipliers.allFinite() ||
                (multipliers.array() < -1e-10).any()) return;
            Point3 point = target;
            for (std::size_t i = 0; i < rank; ++i)
                point += multipliers[i]*region.constraints[active[i]].normal;
            if (!feasible(point)) return;
            const Scalar distance = (point-target).squaredNorm();
            if (distance < best_distance)
            {
                best = point;
                best_distance = distance;
            }
        };
        if (!best)
        {
            for (std::size_t i = 0; i < count; ++i)
                considerActiveSet({i,0,0},1);
            for (std::size_t i = 0; i < count; ++i)
                for (std::size_t j = i+1; j < count; ++j)
                    considerActiveSet({i,j,0},2);
            for (std::size_t i = 0; i < count; ++i)
                for (std::size_t j = i+1; j < count; ++j)
                    for (std::size_t k = j+1; k < count; ++k)
                        considerActiveSet({i,j,k},3);
        }
        if (!best)
        {
            output.status = ApexSolverStatus::EmptyFeasibleRegion;
            return output;
        }
        output.apex = center+length*(*best);
        output.minimum_slack = region.minimumSlack(*output.apex);
        output.status = validQuality(q,*output.apex)
            ? ApexSolverStatus::Valid : ApexSolverStatus::QualityRejected;
        return output;
    }

    std::vector<Point3> projectPyramidApexToPlaneSides(
        const ApexSolverInput &input,
        const Point3 &target,
        const std::array<Point3,3> &plane,
        std::size_t maximum_candidates)
    {
        if (maximum_candidates == 0 || !target.allFinite()) return {};
        const Vector3 normal = (plane[1]-plane[0]).cross(plane[2]-plane[0]);
        const Scalar norm = normal.norm();
        if (!std::isfinite(norm) || norm <= 1e-14) return {};
        Scalar characteristic = 0;
        for (std::size_t i = 0; i < 4; ++i)
            characteristic += (input.quad[(i+1)%4]-input.quad[i]).norm();
        characteristic /= 4;
        constexpr std::array<Scalar,7> margins{{
            Scalar{1e-9},Scalar{0.001},Scalar{0.01},Scalar{0.05},
            Scalar{0.1},Scalar{0.25},Scalar{0.5}}};
        std::vector<Point3> result;
        for (const Scalar margin : margins)
        {
            for (const Scalar side : {Scalar{1},Scalar{-1}})
            {
                ApexSolverInput constrained = input;
                constrained.target = target;
                const Vector3 unit = side*normal/norm;
                constrained.additional_constraints.push_back(ApexConstraint{
                    unit,unit.dot(plane[0])+margin*characteristic});
                const auto solved = solvePyramidApex(constrained);
                if (solved.status == ApexSolverStatus::Valid && solved.apex)
                    result.push_back(*solved.apex);
                if (result.size() >= maximum_candidates) return result;
            }
        }
        return result;
    }
}
