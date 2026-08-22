#include <boundary_mesh/io/legacy_vtk_writer.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace boundary_mesh
{
    namespace
    {
        struct CellView
        {
            std::vector<VertexId> vertex_ids;
            int vtk_type{};
        };

        CellView surfaceCell(const SurfaceFace &face)
        {
            return std::visit(
                [](const auto &value)
                {
                    using Face = std::decay_t<decltype(value)>;
                    return CellView{
                        {value.vertex_ids.begin(), value.vertex_ids.end()},
                        std::is_same_v<Face, Triangle> ? 5 : 9};
                },
                face);
        }

        CellView volumeCell(const VolumeCell &cell)
        {
            return std::visit(
                [](const auto &value)
                {
                    using Cell = std::decay_t<decltype(value)>;
                    int type = 12;
                    if constexpr (std::is_same_v<Cell, Tetra>) type = 10;
                    else if constexpr (std::is_same_v<Cell, Pyramid>)
                        type = 14;
                    else if constexpr (std::is_same_v<Cell, Prism>)
                        type = 13;
                    return CellView{
                        {value.vertex_ids.begin(), value.vertex_ids.end()},
                        type};
                },
                cell);
        }

        VtkWriteStatus writeCells(
            const std::filesystem::path &path,
            const std::vector<Point3> &vertices,
            const std::vector<CellView> &cells)
        {
            const std::uint64_t maximum =
                std::numeric_limits<std::uint32_t>::max();
            if (vertices.size() > maximum || cells.size() > maximum)
            {
                return VtkWriteStatus::failure(
                    {VtkWriteErrorCode::CountOverflow, path, 0, 0});
            }

            std::uint64_t connectivity_count = 0;
            for (std::size_t cell_index = 0;
                 cell_index < cells.size();
                 ++cell_index)
            {
                const CellView &cell = cells[cell_index];
                connectivity_count += cell.vertex_ids.size() + 1;
                if (connectivity_count > maximum)
                {
                    return VtkWriteStatus::failure(
                        {VtkWriteErrorCode::CountOverflow,
                         path,
                         cell_index,
                         0});
                }
                for (const VertexId vertex_id : cell.vertex_ids)
                {
                    if (static_cast<std::size_t>(vertex_id) >=
                        vertices.size())
                    {
                        return VtkWriteStatus::failure(
                            {VtkWriteErrorCode::InvalidVertexReference,
                             path,
                             cell_index,
                             vertex_id});
                    }
                }
            }

            std::filesystem::path temporary = path;
            temporary += ".tmp";
            std::error_code file_error;
            std::filesystem::remove(temporary, file_error);
            file_error.clear();
            std::ofstream output(
                temporary,
                std::ios::binary | std::ios::trunc);
            if (!output.is_open())
            {
                return VtkWriteStatus::failure(
                    {VtkWriteErrorCode::FileOpenFailure, path, 0, 0});
            }

            output << "# vtk DataFile Version 3.0\n"
                   << "BoundaryMesh\n"
                   << "ASCII\n"
                   << "DATASET UNSTRUCTURED_GRID\n"
                   << "POINTS " << vertices.size() << " double\n"
                   << std::setprecision(
                          std::numeric_limits<Scalar>::max_digits10);
            for (const Point3 &point : vertices)
            {
                output << point.x() << ' ' << point.y() << ' '
                       << point.z() << '\n';
            }
            output << "CELLS " << cells.size() << ' '
                   << connectivity_count << '\n';
            for (const CellView &cell : cells)
            {
                output << cell.vertex_ids.size();
                for (const VertexId vertex_id : cell.vertex_ids)
                {
                    output << ' ' << vertex_id;
                }
                output << '\n';
            }
            output << "CELL_TYPES " << cells.size() << '\n';
            for (const CellView &cell : cells)
            {
                output << cell.vtk_type << '\n';
            }
            output.close();
            if (!output)
            {
                std::filesystem::remove(temporary, file_error);
                return VtkWriteStatus::failure(
                    {VtkWriteErrorCode::WriteFailure, path, 0, 0});
            }

            if (std::filesystem::exists(path, file_error))
            {
                file_error.clear();
                std::filesystem::remove(path, file_error);
                if (file_error)
                {
                    std::filesystem::remove(temporary, file_error);
                    return VtkWriteStatus::failure(
                        {VtkWriteErrorCode::ReplaceFailure, path, 0, 0});
                }
            }
            file_error.clear();
            std::filesystem::rename(temporary, path, file_error);
            if (file_error)
            {
                std::error_code cleanup_error;
                std::filesystem::remove(temporary, cleanup_error);
                return VtkWriteStatus::failure(
                    {VtkWriteErrorCode::ReplaceFailure, path, 0, 0});
            }
            return VtkWriteStatus::success(std::monostate{});
        }
    }

    VtkWriteStatus writeLegacyVtk(
        const std::filesystem::path &path,
        const SurfaceMesh &mesh)
    {
        std::vector<CellView> cells;
        cells.reserve(mesh.faces.size());
        for (const SurfaceFace &face : mesh.faces)
        {
            cells.push_back(surfaceCell(face));
        }
        return writeCells(path, mesh.vertices, cells);
    }

    VtkWriteStatus writeLegacyVtk(
        const std::filesystem::path &path,
        const VolumeMesh &mesh)
    {
        std::vector<CellView> cells;
        cells.reserve(mesh.cells.size());
        for (const VolumeCell &cell : mesh.cells)
        {
            cells.push_back(volumeCell(cell));
        }
        return writeCells(path, mesh.vertices, cells);
    }
}
