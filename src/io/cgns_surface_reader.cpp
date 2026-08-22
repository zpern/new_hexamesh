#include <boundary_mesh/io/cgns_surface_reader.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cgnslib.h>

#include <io/boundary_condition_map.hpp>
#include <io/cgns_file.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr std::size_t cgns_name_buffer_size = 33;

        struct ZoneInfo
        {
            int index{}; // CGNS 内部 1-based Zone 下标
            std::uint32_t id{}; // 数字 Zone 名
            cgsize_t vertex_count{}; // Zone 局部顶点数量
        };

        struct PendingFace
        {
            std::uint32_t zone_id{}; // 所属数字 Zone
            cgsize_t element_id{}; // CGNS 1-based 元素编号
            SurfaceFace face; // 尚未加入输出数组的面
            SurfaceBoundaryTag tag; // 与面对应的边界标签
        };

        CgnsSurfaceError makeError(
            CgnsSurfaceErrorCode code,
            const std::filesystem::path &path,
            std::uint32_t zone_id = 0,
            std::uint64_t element_id = 0,
            std::string detail = {})
        {
            CgnsSurfaceError error;
            error.code = code;
            error.path = path;
            error.zone_id = zone_id;
            error.element_id = element_id;
            error.detail = std::move(detail);
            return error;
        }

        CgnsSurfaceError libraryError(
            const std::filesystem::path &path,
            std::uint32_t zone_id = 0,
            std::uint64_t element_id = 0)
        {
            return makeError(
                CgnsSurfaceErrorCode::CgnsLibraryFailure,
                path,
                zone_id,
                element_id,
                cg_get_error());
        }

        bool parseZoneId(
            const char *name,
            std::uint32_t &zone_id)
        {
            const std::string text{name};
            if (text.empty())
            {
                return false;
            }

            std::uint32_t parsed{};
            const auto result = std::from_chars(
                text.data(),
                text.data() + text.size(),
                parsed);
            if (result.ec != std::errc{} ||
                result.ptr != text.data() + text.size() ||
                parsed == 0)
            {
                return false;
            }

            zone_id = parsed;
            return true;
        }

        std::filesystem::path boundaryMapPath(
            const std::filesystem::path &cgns_path)
        {
            return cgns_path.parent_path() /
                   (cgns_path.stem().string() + ".bc.txt");
        }

        bool hasCoordinate(
            int file,
            int base,
            int zone,
            const char *required_name)
        {
            int coordinate_count{};
            if (cg_ncoords(file, base, zone, &coordinate_count) != CG_OK)
            {
                return false;
            }

            for (int coordinate = 1;
                 coordinate <= coordinate_count;
                 ++coordinate)
            {
                CGNS_ENUMT(DataType_t) data_type{};
                char name[cgns_name_buffer_size]{};
                if (cg_coord_info(
                        file,
                        base,
                        zone,
                        coordinate,
                        &data_type,
                        name) != CG_OK)
                {
                    return false;
                }
                if (std::string{name} == required_name)
                {
                    return true;
                }
            }
            return false;
        }
    }

    CgnsSurfaceResult readCgnsSurface(
        const std::filesystem::path &cgns_path)
    {
        int file_number{};
        if (cg_open(
                cgns_path.string().c_str(),
                CG_MODE_READ,
                &file_number) != CG_OK)
        {
            return CgnsSurfaceResult::failure(
                makeError(
                    CgnsSurfaceErrorCode::FileOpenFailure,
                    cgns_path,
                    0,
                    0,
                    cg_get_error()));
        }
        CgnsFile file(file_number);

        int base_count{};
        if (cg_nbases(file.number(), &base_count) != CG_OK)
        {
            return CgnsSurfaceResult::failure(
                libraryError(cgns_path));
        }
        if (base_count != 1)
        {
            return CgnsSurfaceResult::failure(
                makeError(
                    CgnsSurfaceErrorCode::InvalidBaseCount,
                    cgns_path));
        }

        char base_name[cgns_name_buffer_size]{};
        int cell_dimension{};
        int physical_dimension{};
        if (cg_base_read(
                file.number(),
                1,
                base_name,
                &cell_dimension,
                &physical_dimension) != CG_OK)
        {
            return CgnsSurfaceResult::failure(
                libraryError(cgns_path));
        }
        if (cell_dimension != 2 || physical_dimension != 3)
        {
            return CgnsSurfaceResult::failure(
                makeError(
                    CgnsSurfaceErrorCode::InvalidDimensions,
                    cgns_path));
        }

        int zone_count{};
        if (cg_nzones(file.number(), 1, &zone_count) != CG_OK)
        {
            return CgnsSurfaceResult::failure(
                libraryError(cgns_path));
        }

        std::vector<ZoneInfo> zones;
        zones.reserve(static_cast<std::size_t>(zone_count));
        std::unordered_set<std::uint32_t> unique_zone_ids;
        for (int zone_index = 1;
             zone_index <= zone_count;
             ++zone_index)
        {
            char zone_name[cgns_name_buffer_size]{};
            // Structured Zone 最多写入 3 * cell_dimension 项；先安全读取，
            // 再根据 ZoneType 拒绝本读取器不支持的类型。
            cgsize_t size[9]{};
            if (cg_zone_read(
                    file.number(),
                    1,
                    zone_index,
                    zone_name,
                    size) != CG_OK)
            {
                return CgnsSurfaceResult::failure(
                    libraryError(cgns_path));
            }

            std::uint32_t zone_id{};
            if (!parseZoneId(zone_name, zone_id) ||
                !unique_zone_ids.insert(zone_id).second)
            {
                return CgnsSurfaceResult::failure(
                    makeError(
                        CgnsSurfaceErrorCode::InvalidZoneId,
                        cgns_path,
                        zone_id));
            }

            CGNS_ENUMT(ZoneType_t) zone_type{};
            if (cg_zone_type(
                    file.number(),
                    1,
                    zone_index,
                    &zone_type) != CG_OK)
            {
                return CgnsSurfaceResult::failure(
                    libraryError(cgns_path, zone_id));
            }
            if (zone_type != CGNS_ENUMV(Unstructured))
            {
                return CgnsSurfaceResult::failure(
                    makeError(
                        CgnsSurfaceErrorCode::InvalidZoneType,
                        cgns_path,
                        zone_id));
            }

            zones.push_back(ZoneInfo{
                zone_index,
                zone_id,
                size[0]});
        }

        std::sort(
            zones.begin(),
            zones.end(),
            [](const ZoneInfo &left, const ZoneInfo &right)
            {
                return left.id < right.id;
            });

        std::vector<std::uint32_t> zone_ids;
        zone_ids.reserve(zones.size());
        for (const auto &zone : zones)
        {
            zone_ids.push_back(zone.id);
        }

        const auto map_path = boundaryMapPath(cgns_path);
        const auto boundary_map =
            readBoundaryConditionMap(map_path, zone_ids);
        if (!boundary_map.hasValue())
        {
            auto error = makeError(
                CgnsSurfaceErrorCode::BoundaryMapFailure,
                map_path);
            error.boundary_map_error = boundary_map.error();
            return CgnsSurfaceResult::failure(std::move(error));
        }

        SurfaceMesh mesh;
        std::vector<PendingFace> pending_faces;
        for (const auto &zone : zones)
        {
            if (zone.vertex_count < 0 ||
                static_cast<unsigned long long>(zone.vertex_count) >
                    static_cast<unsigned long long>(
                        std::numeric_limits<VertexId>::max()) + 1ULL ||
                mesh.vertices.size() >
                    static_cast<std::size_t>(
                        std::numeric_limits<VertexId>::max()) + 1ULL -
                        static_cast<std::size_t>(zone.vertex_count))
            {
                return CgnsSurfaceResult::failure(
                    makeError(
                        CgnsSurfaceErrorCode::VertexIdOverflow,
                        cgns_path,
                        zone.id));
            }

            constexpr std::array<const char *, 3> coordinate_names{
                "CoordinateX",
                "CoordinateY",
                "CoordinateZ"};
            for (const char *coordinate_name : coordinate_names)
            {
                if (!hasCoordinate(
                        file.number(),
                        1,
                        zone.index,
                        coordinate_name))
                {
                    return CgnsSurfaceResult::failure(
                        makeError(
                            CgnsSurfaceErrorCode::MissingCoordinate,
                            cgns_path,
                            zone.id,
                            0,
                            coordinate_name));
                }
            }

            const auto count =
                static_cast<std::size_t>(zone.vertex_count);
            std::array<std::vector<double>, 3> coordinates{
                std::vector<double>(count),
                std::vector<double>(count),
                std::vector<double>(count)};
            const cgsize_t range_min[1]{1};
            const cgsize_t range_max[1]{zone.vertex_count};
            for (std::size_t axis = 0;
                 axis < coordinate_names.size();
                 ++axis)
            {
                if (cg_coord_read(
                        file.number(),
                        1,
                        zone.index,
                        coordinate_names[axis],
                        CGNS_ENUMV(RealDouble),
                        range_min,
                        range_max,
                        coordinates[axis].data()) != CG_OK)
                {
                    return CgnsSurfaceResult::failure(
                        libraryError(cgns_path, zone.id));
                }
            }

            const auto vertex_offset = mesh.vertices.size();
            for (std::size_t vertex = 0;
                 vertex < count;
                 ++vertex)
            {
                const Point3 point{
                    coordinates[0][vertex],
                    coordinates[1][vertex],
                    coordinates[2][vertex]};
                if (!point.allFinite())
                {
                    return CgnsSurfaceResult::failure(
                        makeError(
                            CgnsSurfaceErrorCode::NonFiniteCoordinate,
                            cgns_path,
                            zone.id));
                }
                mesh.vertices.push_back(point);
            }

            const auto *zone_tag =
                boundary_map.value().find(zone.id);
            if (zone_tag == nullptr)
            {
                return CgnsSurfaceResult::failure(
                    makeError(
                        CgnsSurfaceErrorCode::BoundaryMapFailure,
                        map_path,
                        zone.id));
            }

            int section_count{};
            if (cg_nsections(
                    file.number(),
                    1,
                    zone.index,
                    &section_count) != CG_OK)
            {
                return CgnsSurfaceResult::failure(
                    libraryError(cgns_path, zone.id));
            }

            for (int section = 1;
                 section <= section_count;
                 ++section)
            {
                char section_name[cgns_name_buffer_size]{};
                CGNS_ENUMT(ElementType_t) element_type{};
                cgsize_t start{};
                cgsize_t end{};
                int boundary_count{};
                int parent_flag{};
                if (cg_section_read(
                        file.number(),
                        1,
                        zone.index,
                        section,
                        section_name,
                        &element_type,
                        &start,
                        &end,
                        &boundary_count,
                        &parent_flag) != CG_OK)
                {
                    return CgnsSurfaceResult::failure(
                        libraryError(cgns_path, zone.id));
                }

                int vertices_per_face{};
                if (element_type == CGNS_ENUMV(TRI_3))
                {
                    vertices_per_face = 3;
                }
                else if (element_type == CGNS_ENUMV(QUAD_4))
                {
                    vertices_per_face = 4;
                }
                else
                {
                    return CgnsSurfaceResult::failure(
                        makeError(
                            CgnsSurfaceErrorCode::UnsupportedElementType,
                            cgns_path,
                            zone.id,
                            static_cast<std::uint64_t>(start)));
                }

                const auto element_count =
                    static_cast<std::size_t>(end - start + 1);
                std::vector<cgsize_t> connectivity(
                    element_count *
                    static_cast<std::size_t>(vertices_per_face));
                if (cg_elements_read(
                        file.number(),
                        1,
                        zone.index,
                        section,
                        connectivity.data(),
                        nullptr) != CG_OK)
                {
                    return CgnsSurfaceResult::failure(
                        libraryError(cgns_path, zone.id));
                }

                for (std::size_t local_element = 0;
                     local_element < element_count;
                     ++local_element)
                {
                    std::array<VertexId, 4> global_ids{};
                    for (int corner = 0;
                         corner < vertices_per_face;
                         ++corner)
                    {
                        const cgsize_t local_id =
                            connectivity[
                                local_element *
                                    static_cast<std::size_t>(
                                        vertices_per_face) +
                                static_cast<std::size_t>(corner)];
                        if (local_id < 1 ||
                            local_id > zone.vertex_count)
                        {
                            return CgnsSurfaceResult::failure(
                                makeError(
                                    CgnsSurfaceErrorCode::InvalidElementReference,
                                    cgns_path,
                                    zone.id,
                                    static_cast<std::uint64_t>(
                                        start +
                                        static_cast<cgsize_t>(local_element))));
                        }
                        global_ids[static_cast<std::size_t>(corner)] =
                            static_cast<VertexId>(
                                vertex_offset +
                                static_cast<std::size_t>(local_id - 1));
                    }

                    SurfaceFace face;
                    if (vertices_per_face == 3)
                    {
                        face = Triangle{{
                            global_ids[0],
                            global_ids[1],
                            global_ids[2]}};
                    }
                    else
                    {
                        face = Quad{{
                            global_ids[0],
                            global_ids[1],
                            global_ids[2],
                            global_ids[3]}};
                    }
                    pending_faces.push_back(PendingFace{
                        zone.id,
                        start + static_cast<cgsize_t>(local_element),
                        std::move(face),
                        SurfaceBoundaryTag{
                            zone_tag->kind,
                            zone_tag->region_id}});
                }
            }
        }

        if (pending_faces.size() >
            static_cast<std::size_t>(
                std::numeric_limits<SurfaceFaceId>::max()) + 1ULL)
        {
            return CgnsSurfaceResult::failure(
                makeError(
                    CgnsSurfaceErrorCode::FaceIdOverflow,
                    cgns_path));
        }

        std::sort(
            pending_faces.begin(),
            pending_faces.end(),
            [](const PendingFace &left, const PendingFace &right)
            {
                return std::pair{
                           left.zone_id,
                           left.element_id} <
                       std::pair{
                           right.zone_id,
                           right.element_id};
            });
        mesh.faces.reserve(pending_faces.size());
        mesh.face_tags.reserve(pending_faces.size());
        for (auto &pending : pending_faces)
        {
            mesh.faces.push_back(std::move(pending.face));
            mesh.face_tags.push_back(pending.tag);
        }

        return CgnsSurfaceResult::success(std::move(mesh));
    }
}
