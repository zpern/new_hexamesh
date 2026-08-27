#pragma once

#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    // Preserve BLMesh MNormal's geometric adjacency semantics: coincident
    // coordinates, rather than layer/topology identifiers, define shared vertices.
    bool blmeshTrianglesIntersect(
        const TrianglePoints &first,
        const TrianglePoints &second);
}
