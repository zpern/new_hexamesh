#include <algorithm>

#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool lessKey(
            const LayerQuadFaceKey &left,
            const LayerQuadFaceKey &right)
        {
            return left.source_face_id < right.source_face_id ||
                   (left.source_face_id == right.source_face_id &&
                    left.layer < right.layer);
        }
    }

    Result<QuadDiagonal, LayerQuadDiagonalError>
    LayerQuadDiagonalTable::resolve(
        LayerQuadFaceKey key,
        const OrientedQuad &quad,
        std::optional<QuadDiagonal> forced,
        Scalar length_tolerance)
    {
        using ResolveResult = Result<
            QuadDiagonal, LayerQuadDiagonalError>;
        const auto position = std::lower_bound(
            entries_.begin(), entries_.end(), key,
            [](const Entry &entry, const LayerQuadFaceKey &value)
            { return lessKey(entry.key, value); });
        if (position != entries_.end() &&
            !lessKey(key, position->key) &&
            !lessKey(position->key, key))
        {
            if (forced.has_value() && *forced != position->diagonal)
                return ResolveResult::failure(
                    LayerQuadDiagonalError{
                        ConflictingLayerQuadDiagonal{
                            key, position->diagonal, *forced}});
            return ResolveResult::success(position->diagonal);
        }

        QuadDiagonal diagonal{};
        if (forced.has_value())
            diagonal = *forced;
        else
        {
            const auto selected = chooseQuadDiagonal(
                quad, length_tolerance);
            if (!selected.hasValue())
                return ResolveResult::failure(
                    LayerQuadDiagonalError{selected.error()});
            diagonal = selected.value().diagonal;
        }
        entries_.insert(position, Entry{key, diagonal});
        return ResolveResult::success(diagonal);
    }
}
