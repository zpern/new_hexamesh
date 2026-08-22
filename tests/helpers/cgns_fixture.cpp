#include <helpers/cgns_fixture.hpp>

#include <stdexcept>
#include <limits>

#include <cgnslib.h>

namespace boundary_mesh::test
{
    namespace
    {
        void requireCgns(int status)
        {
            if (status != CG_OK)
            {
                throw std::runtime_error(cg_get_error());
            }
        }
    }

    void writeSingleZoneMixedSurface(
        const std::filesystem::path &path)
    {
        writeSingleZoneSurface(path, SingleZoneFixtureOptions{});
    }

    void writeSingleZoneSurface(
        const std::filesystem::path &path,
        const SingleZoneFixtureOptions &options)
    {
        int file{};
        requireCgns(cg_open(
            path.string().c_str(),
            CG_MODE_WRITE,
            &file));

        int base{};
        requireCgns(cg_base_write(
            file,
            "Surface",
            options.cell_dimension,
            options.physical_dimension,
            &base));

        const cgsize_t unstructured_size[3]{5, 2, 0};
        const cgsize_t structured_size[6]{
            2, 2,
            1, 1,
            0, 0};
        int zone{};
        requireCgns(cg_zone_write(
            file,
            base,
            options.zone_name.c_str(),
            options.zone_kind == FixtureZoneKind::Unstructured
                ? unstructured_size
                : structured_size,
            options.zone_kind == FixtureZoneKind::Unstructured
                ? CGNS_ENUMV(Unstructured)
                : CGNS_ENUMV(Structured),
            &zone));

        if (options.zone_kind == FixtureZoneKind::Structured)
        {
            requireCgns(cg_close(file));
            return;
        }

        const double exceptional =
            options.write_non_finite_coordinate
                ? std::numeric_limits<double>::quiet_NaN()
                : 0.0;
        const double x[5]{exceptional, 1.0, 1.0, 0.0, 2.0};
        const double y[5]{0.0, 0.0, 1.0, 1.0, 0.0};
        const double z[5]{0.0, 0.0, 0.0, 0.0, 0.0};
        int coordinate{};
        requireCgns(cg_coord_write(
            file, base, zone, CGNS_ENUMV(RealDouble),
            "CoordinateX", x, &coordinate));
        requireCgns(cg_coord_write(
            file, base, zone, CGNS_ENUMV(RealDouble),
            "CoordinateY", y, &coordinate));
        if (options.write_coordinate_z)
        {
            requireCgns(cg_coord_write(
                file, base, zone, CGNS_ENUMV(RealDouble),
                "CoordinateZ", z, &coordinate));
        }

        int section{};
        if (options.element_kind ==
            FixtureElementKind::TriangleAndQuad)
        {
            const cgsize_t triangle[3]{1, 2, 3};
            const cgsize_t quad[4]{2, 5, 3, 4};
            requireCgns(cg_section_write(
                file, base, zone, "Triangle", CGNS_ENUMV(TRI_3),
                1, 1, 0, triangle, &section));
            requireCgns(cg_section_write(
                file, base, zone, "Quad", CGNS_ENUMV(QUAD_4),
                2, 2, 0, quad, &section));
        }
        else if (options.element_kind == FixtureElementKind::Line)
        {
            const cgsize_t line[2]{1, 2};
            requireCgns(cg_section_write(
                file, base, zone, "Line", CGNS_ENUMV(BAR_2),
                1, 1, 0, line, &section));
        }
        else
        {
            const cgsize_t triangle[3]{1, 2, 6};
            requireCgns(cg_section_write(
                file, base, zone, "Triangle", CGNS_ENUMV(TRI_3),
                1, 1, 0, triangle, &section));
        }

        requireCgns(cg_close(file));
    }

    void writeEmptyCgns(
        const std::filesystem::path &path)
    {
        int file{};
        requireCgns(cg_open(
            path.string().c_str(), CG_MODE_WRITE, &file));
        requireCgns(cg_close(file));
    }

    void writeTwoBaseCgns(
        const std::filesystem::path &path)
    {
        int file{};
        requireCgns(cg_open(
            path.string().c_str(), CG_MODE_WRITE, &file));
        int base{};
        requireCgns(cg_base_write(file, "First", 2, 3, &base));
        requireCgns(cg_base_write(file, "Second", 2, 3, &base));
        requireCgns(cg_close(file));
    }

    void writeTwoZoneConnectedSurface(
        const std::filesystem::path &path,
        bool reverse_point_order,
        bool mismatch_coordinate)
    {
        int file{};
        requireCgns(cg_open(
            path.string().c_str(), CG_MODE_WRITE, &file));
        int base{};
        requireCgns(cg_base_write(file, "Surface", 2, 3, &base));

        const cgsize_t size[3]{4, 1, 0};
        int zone1{};
        int zone2{};
        requireCgns(cg_zone_write(
            file, base, "1", size,
            CGNS_ENUMV(Unstructured), &zone1));
        requireCgns(cg_zone_write(
            file, base, "2", size,
            CGNS_ENUMV(Unstructured), &zone2));

        const double x1[4]{0.0, 1.0, 1.0, 0.0};
        const double y1[4]{0.0, 0.0, 1.0, 1.0};
        const double x2[4]{
            mismatch_coordinate ? 1.25 : 1.0,
            2.0,
            2.0,
            1.0};
        const double y2[4]{0.0, 0.0, 1.0, 1.0};
        const double z[4]{0.0, 0.0, 0.0, 0.0};
        const auto write_coordinates =
            [&](int zone, const double *x, const double *y)
            {
                int coordinate{};
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateX", x, &coordinate));
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateY", y, &coordinate));
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateZ", z, &coordinate));
            };
        write_coordinates(zone1, x1, y1);
        write_coordinates(zone2, x2, y2);

        const cgsize_t quad[4]{1, 2, 3, 4};
        int section{};
        requireCgns(cg_section_write(
            file, base, zone1, "Quad", CGNS_ENUMV(QUAD_4),
            1, 1, 0, quad, &section));
        requireCgns(cg_section_write(
            file, base, zone2, "Quad", CGNS_ENUMV(QUAD_4),
            1, 1, 0, quad, &section));

        const cgsize_t points_forward[2]{2, 3};
        const cgsize_t donors_forward[2]{1, 4};
        const cgsize_t points_reverse[2]{3, 2};
        const cgsize_t donors_reverse[2]{4, 1};
        int connection{};
        requireCgns(cg_conn_write(
            file,
            base,
            zone1,
            "to-2",
            CGNS_ENUMV(Vertex),
            CGNS_ENUMV(Abutting1to1),
            CGNS_ENUMV(PointList),
            2,
            reverse_point_order
                ? points_reverse
                : points_forward,
            "2",
            CGNS_ENUMV(Unstructured),
            CGNS_ENUMV(PointListDonor),
            CGNS_ENUMV(LongInteger),
            2,
            reverse_point_order
                ? donors_reverse
                : donors_forward,
            &connection));

        requireCgns(cg_close(file));
    }

    void writeClosedCubeSurface(
        const std::filesystem::path &path,
        bool reverse_zone_order,
        bool reverse_point_order)
    {
        int file{};
        requireCgns(cg_open(
            path.string().c_str(), CG_MODE_WRITE, &file));
        int base{};
        requireCgns(cg_base_write(file, "Surface", 2, 3, &base));

        const cgsize_t wall_size[3]{4, 1, 0};
        const cgsize_t far_size[3]{8, 5, 0};
        int wall_zone{};
        int far_zone{};
        const auto write_wall_zone = [&]
        {
            requireCgns(cg_zone_write(
                file, base, "1", wall_size,
                CGNS_ENUMV(Unstructured), &wall_zone));
        };
        const auto write_far_zone = [&]
        {
            requireCgns(cg_zone_write(
                file, base, "2", far_size,
                CGNS_ENUMV(Unstructured), &far_zone));
        };
        if (reverse_zone_order)
        {
            write_far_zone();
            write_wall_zone();
        }
        else
        {
            write_wall_zone();
            write_far_zone();
        }

        const double wall_x[4]{0.0, 1.0, 1.0, 0.0};
        const double wall_y[4]{0.0, 0.0, 1.0, 1.0};
        const double wall_z[4]{0.0, 0.0, 0.0, 0.0};
        const double far_x[8]{0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0};
        const double far_y[8]{0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0};
        const double far_z[8]{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
        const auto write_coordinates =
            [&](int zone, const double *x, const double *y,
                const double *z)
            {
                int coordinate{};
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateX", x, &coordinate));
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateY", y, &coordinate));
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateZ", z, &coordinate));
            };
        write_coordinates(wall_zone, wall_x, wall_y, wall_z);
        write_coordinates(far_zone, far_x, far_y, far_z);

        const cgsize_t wall[4]{1, 4, 3, 2};
        const cgsize_t far_faces[20]{
            5, 6, 7, 8,
            1, 2, 6, 5,
            2, 3, 7, 6,
            3, 4, 8, 7,
            4, 1, 5, 8};
        int section{};
        requireCgns(cg_section_write(
            file, base, wall_zone, "Wall", CGNS_ENUMV(QUAD_4),
            1, 1, 0, wall, &section));
        requireCgns(cg_section_write(
            file, base, far_zone, "Far", CGNS_ENUMV(QUAD_4),
            1, 5, 0, far_faces, &section));

        const cgsize_t points_forward[4]{1, 2, 3, 4};
        const cgsize_t points_reverse[4]{4, 3, 2, 1};
        const auto *points = reverse_point_order
            ? points_reverse
            : points_forward;
        int connection{};
        requireCgns(cg_conn_write(
            file, base, wall_zone, "to-2",
            CGNS_ENUMV(Vertex), CGNS_ENUMV(Abutting1to1),
            CGNS_ENUMV(PointList), 4, points, "2",
            CGNS_ENUMV(Unstructured), CGNS_ENUMV(PointListDonor),
            CGNS_ENUMV(LongInteger), 4, points, &connection));

        requireCgns(cg_close(file));
    }

    void writeTwoZoneOrderVariantSurface(
        const std::filesystem::path &path,
        bool reverse_zone_order,
        bool reverse_section_order,
        bool reverse_connection_order)
    {
        int file{};
        requireCgns(cg_open(
            path.string().c_str(), CG_MODE_WRITE, &file));
        int base{};
        requireCgns(cg_base_write(file, "Surface", 2, 3, &base));

        const cgsize_t size[3]{4, 2, 0};
        int zone1{};
        int zone2{};
        const auto write_zone = [&](const char *name, int &zone)
        {
            requireCgns(cg_zone_write(
                file, base, name, size,
                CGNS_ENUMV(Unstructured), &zone));
        };
        if (reverse_zone_order)
        {
            write_zone("2", zone2);
            write_zone("1", zone1);
        }
        else
        {
            write_zone("1", zone1);
            write_zone("2", zone2);
        }

        const double x1[4]{0.0, 1.0, 1.0, 0.0};
        const double y1[4]{0.0, 0.0, 1.0, 1.0};
        const double x2[4]{1.0, 2.0, 2.0, 1.0};
        const double y2[4]{0.0, 0.0, 1.0, 1.0};
        const double z[4]{0.0, 0.0, 0.0, 0.0};
        const auto write_coordinates =
            [&](int zone, const double *x, const double *y)
            {
                int coordinate{};
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateX", x, &coordinate));
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateY", y, &coordinate));
                requireCgns(cg_coord_write(
                    file, base, zone, CGNS_ENUMV(RealDouble),
                    "CoordinateZ", z, &coordinate));
            };
        write_coordinates(zone1, x1, y1);
        write_coordinates(zone2, x2, y2);

        const cgsize_t first_triangle[3]{1, 2, 3};
        const cgsize_t second_triangle[3]{1, 3, 4};
        const auto write_sections = [&](int zone)
        {
            int section{};
            const auto write_first = [&]
            {
                requireCgns(cg_section_write(
                    file, base, zone, "Triangle-1",
                    CGNS_ENUMV(TRI_3), 1, 1, 0,
                    first_triangle, &section));
            };
            const auto write_second = [&]
            {
                requireCgns(cg_section_write(
                    file, base, zone, "Triangle-2",
                    CGNS_ENUMV(TRI_3), 2, 2, 0,
                    second_triangle, &section));
            };
            if (reverse_section_order)
            {
                write_second();
                write_first();
            }
            else
            {
                write_first();
                write_second();
            }
        };
        write_sections(zone1);
        write_sections(zone2);

        const cgsize_t point_lower[1]{2};
        const cgsize_t donor_lower[1]{1};
        const cgsize_t point_upper[1]{3};
        const cgsize_t donor_upper[1]{4};
        const auto write_connection =
            [&](const char *name,
                const cgsize_t *points,
                const cgsize_t *donors)
            {
                int connection{};
                requireCgns(cg_conn_write(
                    file, base, zone1, name,
                    CGNS_ENUMV(Vertex),
                    CGNS_ENUMV(Abutting1to1),
                    CGNS_ENUMV(PointList), 1, points, "2",
                    CGNS_ENUMV(Unstructured),
                    CGNS_ENUMV(PointListDonor),
                    CGNS_ENUMV(LongInteger), 1, donors,
                    &connection));
            };
        if (reverse_connection_order)
        {
            write_connection("upper", point_upper, donor_upper);
            write_connection("lower", point_lower, donor_lower);
        }
        else
        {
            write_connection("lower", point_lower, donor_lower);
            write_connection("upper", point_upper, donor_upper);
        }

        requireCgns(cg_close(file));
    }
}
