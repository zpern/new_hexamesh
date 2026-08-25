#include <boundary_mesh/growth/blmesh_intersection_checker.hpp>

#include "geom_func.h"

namespace boundary_mesh
{
    namespace
    {
        bool samePoint(const Point3 &lhs, const Point3 &rhs)
        {
            return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
        }

        void copyPoint(const Point3 &source, double destination[3])
        {
            for (int coordinate = 0; coordinate < 3; ++coordinate)
                destination[coordinate] = source[coordinate];
        }
    }

    bool blmeshTrianglesIntersect(
        const TrianglePoints &first,
        const TrianglePoints &second)
    {
        int same_count = 0;
        int first_shared = 0;
        int second_shared = 0;
        double points[6][3]{};

        for (int first_id = 0; first_id < 3; ++first_id)
        {
            for (int second_id = 0; second_id < 3; ++second_id)
            {
                if (samePoint(first[first_id], second[second_id]))
                {
                    ++same_count;
                    first_shared = first_id;
                    second_shared = second_id;
                }
            }
            copyPoint(first[first_id], points[first_id]);
            copyPoint(second[first_id], points[first_id + 3]);
        }

        if (same_count == 3 || same_count == 2)
            return false;

        if (same_count == 0)
            return TiGER_GEOM_FUNC::tri_tri_overlap_test_3d(
                       points[0], points[1], points[2],
                       points[3], points[4], points[5]) != 0;

        if (same_count != 1)
            return false;

        double line[2][3]{};
        double face[3][3]{};
        int intersection_type = 0;
        int intersection_code = 0;
        double intersection_point[3]{};
        constexpr bool use_epsilon = false;

        for (int coordinate = 0; coordinate < 3; ++coordinate)
        {
            line[0][coordinate] = points[(first_shared + 1) % 3][coordinate];
            line[1][coordinate] = points[(first_shared + 2) % 3][coordinate];
            for (int vertex = 0; vertex < 3; ++vertex)
                face[vertex][coordinate] = points[vertex + 3][coordinate];
        }
        if (TiGER_GEOM_FUNC::lin_tri_intersect3d(
                line, face, &intersection_type, &intersection_code,
                intersection_point, use_epsilon))
            return true;

        for (int coordinate = 0; coordinate < 3; ++coordinate)
        {
            line[0][coordinate] = points[(second_shared + 1) % 3 + 3][coordinate];
            line[1][coordinate] = points[(second_shared + 2) % 3 + 3][coordinate];
            for (int vertex = 0; vertex < 3; ++vertex)
                face[vertex][coordinate] = points[vertex][coordinate];
        }
        return TiGER_GEOM_FUNC::lin_tri_intersect3d(
                   line, face, &intersection_type, &intersection_code,
                   intersection_point, use_epsilon) != 0;
    }
}
