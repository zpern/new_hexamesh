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
}
