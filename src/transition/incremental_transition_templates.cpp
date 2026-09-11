#include <boundary_mesh/transition/incremental_transition_templates.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/LU>

namespace boundary_mesh
{
    namespace
    {
        struct AffineVolumeConstraint
        {
            Vector3 coefficient{Vector3::Zero()};
            Scalar offset{};
        };

        AffineVolumeConstraint volumeConstraint(
            const Point3 &first,
            const Point3 &second,
            const Point3 &third,
            Scalar inverse_scale)
        {
            const Vector3 coefficient =
                (second - first).cross(third - first) /
                Scalar{6} * inverse_scale;
            return {coefficient, -coefficient.dot(first)};
        }

        Scalar marginAt(
            const std::array<AffineVolumeConstraint, 12> &constraints,
            const Point3 &point)
        {
            Scalar margin = std::numeric_limits<Scalar>::infinity();
            for (const auto &constraint : constraints)
                margin = std::min(
                    margin,
                    constraint.coefficient.dot(point) + constraint.offset);
            return margin;
        }
    }

    Scalar quadTopCapAspectRatio(
        const QuadTopCapAspectRatioInput &input)
    {
        Vector3 area_vector = Vector3::Zero();
        for (std::size_t index = 0; index < 4; ++index)
            area_vector += input.bottom[index].cross(
                input.bottom[(index + 1) % 4]);
        area_vector *= Scalar{0.5};
        const Scalar area = area_vector.norm();
        if (!std::isfinite(area) || area <= Scalar{0})
            return Scalar{0};
        const Vector3 normal = area_vector / area;
        Scalar height = std::numeric_limits<Scalar>::infinity();
        for (std::size_t index = 0; index < 4; ++index)
        {
            if (!input.bottom[index].allFinite() ||
                !input.top[index].allFinite())
                return Scalar{0};
            height = std::min(height, std::abs(
                (input.top[index] - input.bottom[index]).dot(normal)));
        }
        const Scalar ratio = height / std::sqrt(area);
        return std::isfinite(ratio) ? ratio : Scalar{0};
    }

    std::optional<Scalar> quadTopCapValidityMargin(
        const std::array<Point3, 4> &bottom,
        const std::array<Point3, 4> &top,
        QuadDiagonal diagonal,
        const Point3 &center)
    {
        std::array<VolumeCell, 7> cells{
            Pyramid{{0,1,2,3,4}},
            Pyramid{{0,4,5,1,6}},
            Pyramid{{1,5,6,2,7}},
            Pyramid{{2,6,7,3,8}},
            Pyramid{{3,7,4,0,9}},
            Tetra{{10,12,11,13}},
            Tetra{{10,13,12,14}}};
        std::vector<Point3> points{
            bottom[0], bottom[1], bottom[2], bottom[3], center,
            top[0], top[1], top[2], top[3], center,
            top[0], top[2], top[1], center, center};
        cells[1] = Pyramid{{0,5,6,1,4}};
        cells[2] = Pyramid{{1,6,7,2,4}};
        cells[3] = Pyramid{{2,7,8,3,4}};
        cells[4] = Pyramid{{3,8,5,0,4}};
        if (diagonal == QuadDiagonal::ZeroTwo)
        {
            cells[5] = Tetra{{5,7,6,4}};
            cells[6] = Tetra{{5,8,7,4}};
        }
        else
        {
            cells[5] = Tetra{{6,8,7,4}};
            cells[6] = Tetra{{6,5,8,4}};
        }
        // Rebuild the compact point list used by the local cells.
        points = {bottom[0], bottom[1], bottom[2], bottom[3], center,
                  top[0], top[1], top[2], top[3]};
        Scalar margin = std::numeric_limits<Scalar>::infinity();
        for (const VolumeCell &cell : cells)
        {
            if (const auto *value = std::get_if<Pyramid>(&cell))
            {
                PyramidPoints p{};
                for (std::size_t i = 0; i < 5; ++i)
                    p[i] = points[value->vertex_ids[i]];
                const auto quality = evaluatePyramid(p);
                if (!quality.hasValue() || quality.value().validity !=
                    VolumeCellValidity::Valid) return std::nullopt;
                margin = std::min(margin, std::min(
                    quality.value().minimum_subtet_signed_volume,
                    std::min(quality.value().minimum_local_jacobian,
                             quality.value().signed_volume)));
            }
            else
            {
                const auto &tetra = std::get<Tetra>(cell);
                TetraPoints p{};
                for (std::size_t i = 0; i < 4; ++i)
                    p[i] = points[tetra.vertex_ids[i]];
                const auto quality = evaluateTetra(p);
                if (!quality.hasValue() || quality.value().validity !=
                    VolumeCellValidity::Valid) return std::nullopt;
                margin = std::min(margin, std::min(
                    quality.value().minimum_subtet_signed_volume,
                    std::min(quality.value().minimum_local_jacobian,
                             quality.value().signed_volume)));
            }
        }
        return std::isfinite(margin) ? std::optional<Scalar>{margin}
                                     : std::nullopt;
    }

    std::optional<Point3> findPositiveQuadTopCapCenter(
        const PositiveQuadTopCapCenterInput &input)
    {
        Point3 minimum = input.bottom[0];
        Point3 maximum = input.bottom[0];
        for (const auto &face : {input.bottom, input.top})
            for (const Point3 &point : face)
            {
                if (!point.allFinite()) return std::nullopt;
                minimum = minimum.cwiseMin(point);
                maximum = maximum.cwiseMax(point);
            }
        const Scalar length = (maximum - minimum).norm();
        if (!std::isfinite(length) || length <= Scalar{0})
            return std::nullopt;
        const Scalar inverse_scale = Scalar{1} /
            (length * length * length);

        const auto &b = input.bottom;
        const auto &t = input.top;
        std::array<AffineVolumeConstraint, 12> constraints{{
            volumeConstraint(b[0], b[1], b[2], inverse_scale),
            volumeConstraint(b[0], b[2], b[3], inverse_scale),
            volumeConstraint(b[0], t[0], t[1], inverse_scale),
            volumeConstraint(b[0], t[1], b[1], inverse_scale),
            volumeConstraint(b[1], t[1], t[2], inverse_scale),
            volumeConstraint(b[1], t[2], b[2], inverse_scale),
            volumeConstraint(b[2], t[2], t[3], inverse_scale),
            volumeConstraint(b[2], t[3], b[3], inverse_scale),
            volumeConstraint(b[3], t[3], t[0], inverse_scale),
            volumeConstraint(b[3], t[0], b[0], inverse_scale),
            input.diagonal == QuadDiagonal::ZeroTwo
                ? volumeConstraint(t[0], t[2], t[1], inverse_scale)
                : volumeConstraint(t[1], t[3], t[2], inverse_scale),
            input.diagonal == QuadDiagonal::ZeroTwo
                ? volumeConstraint(t[0], t[3], t[2], inverse_scale)
                : volumeConstraint(t[1], t[0], t[3], inverse_scale)}};

        Point3 best = Point3::Zero();
        for (const Point3 &point : b) best += point;
        for (const Point3 &point : t) best += point;
        best /= Scalar{8};
        Scalar best_margin = -std::numeric_limits<Scalar>::infinity();
        if (const auto validity = quadTopCapValidityMargin(
                input.bottom, input.top, input.diagonal, best))
            best_margin = *validity;

        for (std::size_t i = 0; i + 3 < constraints.size(); ++i)
            for (std::size_t j = i + 1; j + 2 < constraints.size(); ++j)
                for (std::size_t k = j + 1; k + 1 < constraints.size(); ++k)
                    for (std::size_t l = k + 1; l < constraints.size(); ++l)
                    {
                        Eigen::Matrix<Scalar, 4, 4> matrix;
                        Eigen::Matrix<Scalar, 4, 1> right;
                        const std::array<std::size_t, 4> active{{i,j,k,l}};
                        for (std::size_t row = 0; row < 4; ++row)
                        {
                            const auto &constraint = constraints[active[row]];
                            matrix.template block<1,3>(row,0) =
                                constraint.coefficient.transpose();
                            matrix(row,3) = Scalar{-1};
                            right(row) = -constraint.offset;
                        }
                        const Eigen::FullPivLU<decltype(matrix)> lu(matrix);
                        if (!lu.isInvertible()) continue;
                        const auto solution = lu.solve(right);
                        if (!solution.allFinite()) continue;
                        const Point3 candidate = solution.template head<3>();
                        if (marginAt(constraints, candidate) <=
                            input.volume_tolerance)
                            continue;
                        const auto validity = quadTopCapValidityMargin(
                            input.bottom, input.top, input.diagonal, candidate);
                        if (!validity) continue;
                        const Scalar candidate_margin = *validity;
                        if (candidate_margin > best_margin)
                        {
                            best = candidate;
                            best_margin = candidate_margin;
                        }
                    }

        return std::isfinite(best_margin) &&
                best_margin > input.volume_tolerance
            ? std::optional<Point3>{best}
            : std::nullopt;
    }

    namespace
    {
        bool validIds(
            const std::array<VertexId, 4> &ids,
            const std::vector<Point3> &points)
        {
            for (const VertexId id : ids)
                if (static_cast<std::size_t>(id) >= points.size())
                    return false;
            return true;
        }

        void addMetadata(
            TransitionTemplateOutput &result,
            std::size_t count,
            std::uint32_t layer)
        {
            for (std::size_t index = 0; index < count; ++index)
                result.metadata.push_back({
                    CellRole::LayerTransition,
                    result.source_face_id,
                    layer});
        }
    }

    TransitionTemplateResult
    buildQuadTopCap(const QuadTopCapInput &input)
    {
        using BuildResult = Result<
            TransitionTemplateOutput, TransitionTemplateError>;
        if (input.mesh_vertices == nullptr || input.layer == 0 ||
            !validIds(input.bottom, *input.mesh_vertices) ||
            !validIds(input.top, *input.mesh_vertices))
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});

        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.low_diagonal = input.diagonal;
        std::array<Point3, 4> bottom_points{};
        std::array<Point3, 4> top_points{};
        for (std::size_t index = 0; index < 4; ++index)
        {
            bottom_points[index] = (*input.mesh_vertices)[input.bottom[index]];
            top_points[index] = (*input.mesh_vertices)[input.top[index]];
        }
        const auto center = input.center_point.has_value()
            ? input.center_point
            : findPositiveQuadTopCapCenter({
                bottom_points, top_points, input.diagonal, Scalar{1e-12}});
        if (!center.has_value())
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id, 14}});
        result.created_vertices.push_back(*center);

        const VertexId c = input.center_vertex_id;
        result.volume_cells = {
            Pyramid{{input.bottom[0], input.bottom[1],
                     input.bottom[2], input.bottom[3], c}},
            Pyramid{{input.bottom[0], input.top[0],
                     input.top[1], input.bottom[1], c}},
            Pyramid{{input.bottom[1], input.top[1],
                     input.top[2], input.bottom[2], c}},
            Pyramid{{input.bottom[2], input.top[2],
                     input.top[3], input.bottom[3], c}},
            Pyramid{{input.bottom[3], input.top[3],
                     input.top[0], input.bottom[0], c}}};
        if (input.diagonal == QuadDiagonal::ZeroTwo)
        {
            result.volume_cells.push_back(
                Tetra{{input.top[0], input.top[2], input.top[1], c}});
            result.volume_cells.push_back(
                Tetra{{input.top[0], input.top[3], input.top[2], c}});
            result.top_faces = {
                Triangle{{input.top[0], input.top[1], input.top[2]}},
                Triangle{{input.top[0], input.top[2], input.top[3]}}};
        }
        else
        {
            result.volume_cells.push_back(
                Tetra{{input.top[1], input.top[3], input.top[2], c}});
            result.volume_cells.push_back(
                Tetra{{input.top[1], input.top[0], input.top[3], c}});
            result.top_faces = {
                Triangle{{input.top[1], input.top[2], input.top[3]}},
                Triangle{{input.top[1], input.top[3], input.top[0]}}};
        }
        addMetadata(result, result.volume_cells.size(), input.layer);
        return BuildResult::success(std::move(result));
    }

    TransitionTemplateResult
    buildExternalQuadPatch(const ExternalQuadPatchInput &input)
    {
        using BuildResult = Result<
            TransitionTemplateOutput, TransitionTemplateError>;
        if (input.mesh_vertices == nullptr || input.layer == 0 ||
            !validIds(input.low, *input.mesh_vertices) ||
            !validIds(input.high, *input.mesh_vertices) ||
            input.high_edges.size() > 2 ||
            !std::isfinite(input.distance_scale) ||
            input.distance_scale <= Scalar{0})
            return BuildResult::failure(TransitionTemplateError{
                InvalidTransitionTemplateInput{input.source_face_id, 15}});

        std::vector<std::array<VertexId, 4>> bases;
        bases.push_back(input.low);
        for (const std::size_t edge : input.high_edges)
        {
            if (edge >= 4)
                return BuildResult::failure(TransitionTemplateError{
                    InvalidTransitionTemplateInput{input.source_face_id,16}});
            const std::size_t next = (edge + 1) % 4;
            bases.push_back({input.low[edge], input.high[edge],
                             input.high[next], input.low[next]});
        }

        Point3 anchor = Point3::Zero();
        if (input.high_edges.empty())
        {
            for (const VertexId id : input.low)
                anchor += (*input.mesh_vertices)[id];
            anchor /= Scalar{4};
        }
        else if (input.high_edges.size() == 1)
        {
            const std::size_t edge = input.high_edges[0];
            anchor = ((*input.mesh_vertices)[input.low[edge]] +
                      (*input.mesh_vertices)[input.low[(edge + 1) % 4]]) /
                Scalar{2};
        }
        else
        {
            const std::size_t first = input.high_edges[0];
            const std::size_t second = input.high_edges[1];
            std::optional<std::size_t> common;
            for (const std::size_t vertex : {first, (first + 1) % 4})
                if (vertex == second || vertex == (second + 1) % 4)
                    common = vertex;
            if (!common.has_value())
                return BuildResult::failure(TransitionTemplateError{
                    InvalidTransitionTemplateInput{input.source_face_id,17}});
            anchor = (*input.mesh_vertices)[input.low[*common]];
        }

        Vector3 direction = Vector3::Zero();
        for (const auto &base : bases)
        {
            const Point3 &p0 = (*input.mesh_vertices)[base[0]];
            const Point3 &p1 = (*input.mesh_vertices)[base[1]];
            const Point3 &p2 = (*input.mesh_vertices)[base[2]];
            const Point3 &p3 = (*input.mesh_vertices)[base[3]];
            Vector3 normal = (p1-p0).cross(p2-p0) +
                             (p2-p0).cross(p3-p0);
            const Scalar norm = normal.norm();
            if (!std::isfinite(norm) || norm <= input.length_tolerance)
                return BuildResult::failure(TransitionTemplateError{
                    InvalidTransitionTemplateInput{input.source_face_id,18}});
            direction += normal / norm;
        }
        const Scalar direction_norm = direction.norm();
        if (!std::isfinite(direction_norm) ||
            direction_norm <= input.length_tolerance)
            return BuildResult::failure(TransitionTemplateError{
                InvalidTransitionTemplateInput{input.source_face_id,19}});
        direction /= direction_norm;

        Scalar characteristic = Scalar{0};
        for (std::size_t edge = 0; edge < 4; ++edge)
            characteristic +=
                ((*input.mesh_vertices)[input.low[(edge+1)%4]] -
                 (*input.mesh_vertices)[input.low[edge]]).norm();
        characteristic /= Scalar{4};
        const Point3 apex = input.apex_point.has_value()
            ? *input.apex_point
            : anchor + input.distance_scale * characteristic * direction;
        if (!apex.allFinite())
            return BuildResult::failure(TransitionTemplateError{
                InvalidTransitionTemplateInput{input.source_face_id,20}});

        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.created_vertices.push_back(apex);
        for (const auto &base : bases)
        {
            PyramidPoints pyramid_points{{
                (*input.mesh_vertices)[base[0]],
                (*input.mesh_vertices)[base[1]],
                (*input.mesh_vertices)[base[2]],
                (*input.mesh_vertices)[base[3]], apex}};
            const auto evaluation = evaluatePyramid(pyramid_points);
            if (!evaluation.hasValue() ||
                evaluation.value().validity != VolumeCellValidity::Valid)
                return BuildResult::failure(TransitionTemplateError{
                    InvalidTransitionTemplateInput{input.source_face_id,21}});
            result.volume_cells.push_back(Pyramid{{
                base[0],base[1],base[2],base[3],input.apex_vertex_id}});
            result.top_faces.push_back(
                Triangle{{base[0],base[1],input.apex_vertex_id}});
            result.top_faces.push_back(
                Triangle{{base[1],base[2],input.apex_vertex_id}});
            result.top_faces.push_back(
                Triangle{{base[2],base[3],input.apex_vertex_id}});
            result.top_faces.push_back(
                Triangle{{base[3],base[0],input.apex_vertex_id}});
        }
        addMetadata(result, result.volume_cells.size(), input.layer);
        return BuildResult::success(std::move(result));
    }

    TransitionTemplateResult
    buildQuadSideTransition(const QuadSideTransitionInput &input)
    {
        using BuildResult = Result<
            TransitionTemplateOutput, TransitionTemplateError>;
        if (input.high_edge_local_index >= 4)
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});

        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.low_diagonal = input.low_diagonal;
        const std::size_t edge = input.high_edge_local_index;
        const VertexId a = input.low[edge];
        const VertexId b = input.low[(edge + 1) % 4];
        const VertexId d = input.low[(edge + 2) % 4];
        const VertexId c = input.low[(edge + 3) % 4];
        const VertexId e = input.high[edge];
        const VertexId f = input.high[(edge + 1) % 4];
        const bool diagonal_ad =
            (edge % 2 == 0 &&
             input.low_diagonal == QuadDiagonal::ZeroTwo) ||
            (edge % 2 == 1 &&
             input.low_diagonal == QuadDiagonal::OneThree);
        if (diagonal_ad)
        {
            result.volume_cells = {
                Pyramid{{e, f, b, a, d}},
                Tetra{{a, d, c, e}}};
            result.top_faces = {
                Triangle{{a, e, c}}, Triangle{{e, d, c}},
                Triangle{{e, f, d}}, Triangle{{f, b, d}}};
        }
        else
        {
            result.volume_cells = {
                Pyramid{{f, b, a, e, c}},
                Tetra{{b, d, c, f}}};
            result.top_faces = {
                Triangle{{b, f, d}}, Triangle{{f, c, d}},
                Triangle{{f, e, c}}, Triangle{{e, a, c}}};
        }
        addMetadata(
            result, result.volume_cells.size(), input.low_layer + 1);
        return BuildResult::success(std::move(result));
    }

    TransitionTemplateResult
    buildQuadAdjacentSideTransition(
        const QuadAdjacentSideTransitionInput &input)
    {
        using BuildResult = Result<
            TransitionTemplateOutput, TransitionTemplateError>;
        const std::size_t first = input.first_high_edge_local_index;
        const std::size_t second = input.second_high_edge_local_index;
        if (input.mesh_vertices == nullptr || first >= 4 || second >= 4 ||
            !validIds(input.low, *input.mesh_vertices) ||
            !validIds(input.high, *input.mesh_vertices) ||
            ((first + 1) % 4 != second && (second + 1) % 4 != first))
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});
        const std::size_t common =
            (first + 1) % 4 == second ? second : first;
        const QuadDiagonal required = common % 2 == 0
            ? QuadDiagonal::ZeroTwo
            : QuadDiagonal::OneThree;
        if (input.low_diagonal != required)
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});

        const VertexId a = input.low[common];
        const VertexId b = input.low[(common + 1) % 4];
        const VertexId d = input.low[(common + 2) % 4];
        const VertexId c = input.low[(common + 3) % 4];
        const VertexId e = input.high[common];
        const VertexId f = input.high[(common + 1) % 4];
        const VertexId g = input.high[(common + 3) % 4];

        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.low_diagonal = input.low_diagonal;
        result.volume_cells = {
            Pyramid{{a,e,f,b,d}}, Pyramid{{a,c,g,e,d}}};
        result.top_faces = {
            Triangle{{b,f,d}}, Triangle{{e,f,d}},
            Triangle{{e,g,d}}, Triangle{{g,c,d}}};
        addMetadata(result, result.volume_cells.size(),
                    input.low_layer + 1);
        return BuildResult::success(std::move(result));
    }
}
