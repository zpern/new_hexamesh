# Explicit Multi-Normal Transition Stage Design

## Goal

Provide a caller-visible multi-normal stage between initial topology/front construction and regular boundary-layer growth. The stage performs detection, splitting, topology stitching, affected-Quad triangulation, split-vertex displacement, and BLMesh-style tetra-only transition construction. It returns both intermediate volume cells and the transformed surface, and can optionally write both artifacts as Legacy VTK files for debugging.

## Scope

- The caller explicitly invokes `generateMultiNormalTransition()` after building the initial topology/front.
- `generateRegularLayers()` performs regular-layer growth only and never invokes multi-normal processing internally.
- Keep a transformed Quad unchanged when none of its corners is a multi-normal branch vertex.
- Triangulate a transformed Quad when at least one corner is a multi-normal branch vertex.
- Stop emitting `OmittedQuadTransition` diagnostics for affected Quads because they will no longer be omitted.
- Preserve the existing triangle transition decomposition, point-ID ordering, signed-volume correction, and degenerate-tetra filtering.
- Do not add Pyramid, Prism, or Hexa transition cells.
- Merge transition and regular volume meshes only after regular-layer generation completes.

## Public Workflow

The public workflow is explicit:

```cpp
auto transition = generateMultiNormalTransition(
    initial_front, multi_normal_options);

auto regular = generateRegularLayers(
    surface_mesh,
    topology,
    patch,
    transition.value().transformed_front,
    profiles,
    regular_options);

auto combined = mergeMultiNormalAndRegularMeshes(
    transition.value(),
    regular.value().mesh);
```

`RegularLayerGrowthOptions` no longer owns `MultiNormalOptions`. `RegularLayerGrowthError` no longer wraps `MultiNormalError`. The regular generator initializes its own mesh from the supplied front and has no knowledge of pre-transition bottom/upper indexing.

The regular generator replaces its current exact `GrowthPatch` vertex/face equality check with transformed-front validation: every front vertex must refer to a source vertex present in the patch/profile table, and every front face must refer to a source face present in the patch constraint table. Repeated source IDs are valid because split vertices and triangulated Quads intentionally create them.

`generateMultiNormalTransition()` is the single independently callable and testable orchestration function. It returns an unchanged front, an empty transition mesh, and `applied == false` when multi-normal processing is disabled or no split plan is selected.

## Processing Order

1. Build split branch vertices.
2. Remap original Triangle and Quad faces to their branch vertices.
3. Add stitching triangles between adjacent branches.
4. Traverse the resulting front and triangulate affected Quads.
5. Move only multi-normal branch vertices by `transition_height`.
6. Build transition volume cells from all transformed Triangle faces through the existing `addTet()` paths.
7. If debug output is enabled, write the transition volume and transformed surface before returning.

The diagonal decision uses the coordinates from step 4, before the branch vertices are moved.

## Diagonal Selection

For an affected Quad with ordered corners `(v0, v1, v2, v3)`, evaluate both candidates:

- Diagonal `v0-v2`: triangles `(v0, v1, v2)` and `(v0, v2, v3)`.
- Diagonal `v1-v3`: triangles `(v0, v1, v3)` and `(v1, v2, v3)`.

For each candidate, compute the equiangular skewness of both triangles using the existing `triangleEquiangularSkewness()` implementation. Its score is the larger of the two values. Select the candidate with the smaller score.

If the absolute score difference is at most `1e-12`, treat the scores as equal. Use the existing stitching-strip point-ID rule: select the diagonal incident to the smallest of the four topology vertex IDs. Therefore `v0-v2` wins when the smallest ID is at corner 0 or 2; otherwise `v1-v3` wins. This makes the result independent of face traversal order.

If a candidate contains a geometrically invalid triangle and skewness evaluation fails, treat that candidate as unavailable. If both candidates are unavailable, return a diagnosable multi-normal topology error rather than silently emitting an invalid front.

## Metadata

Both triangles produced from a Quad inherit:

- the original Quad's `source_face_id`;
- the original transformed face's source-face origins;
- the original transformed face's source-vertex origins.

Source-level regular-layer accounting continues to record the maximum accepted layer number, so two triangles derived from one source Quad do not count as two layers.

## Transition Volume Behavior

The transition builder receives the affected Quad as two Triangle faces. It applies the existing lower-point merge by `source_vertex_id` and the existing BLMesh-derived tetra-only cases. No Quad-specific volume decomposition remains in this path.

Unaffected Quads remain Quads in `transformed_front`. Because none of their vertices moved during the pre-transition step, they do not enclose a transition volume and require no transition cells.

`OmittedQuadTransition` remains in the public result type for source compatibility but is empty for this workflow.

## Debug Output

Debug output is disabled by default. `MultiNormalOptions` contains a debug-output configuration with:

- `enabled`, default `false`;
- `directory`, used only when enabled;
- transition filename, default `multi_normal_transition.vtk`;
- transformed-front filename, default `multi_normal_front.vtk`.

After successful in-memory construction, `generateMultiNormalTransition()` converts the transformed `GrowthFront` to a `SurfaceMesh` and uses the existing Legacy VTK writer for both artifacts. A write failure returns a multi-normal debug-output error and the caller must not start regular-layer growth.

The Legacy VTK writer is lightweight and does not require CGNS. CMake will build `BoundaryMesh::IO` with `legacy_vtk_writer.cpp` unconditionally; CGNS reader sources and the CGNS link dependency remain conditional on `BOUNDARY_MESH_ENABLE_CGNS_IO`. `BoundaryMesh::BoundaryLayer` may therefore depend on `BoundaryMesh::IO` without disabling the normal `BOUNDARY_MESH_ENABLE_CGNS_IO=OFF` test build.

## Volume Mesh Merge

`MultiNormalTransitionResult` exposes `transformed_front_volume_vertex_ids`, ordered exactly like `transformed_front.vertices`. These IDs identify the upper transition vertices already stored in `transition_cells.vertices`.

`mergeMultiNormalAndRegularMeshes(transition, regular)` returns a new `VolumeMesh`. The regular mesh is required to store the supplied transformed-front vertices first, in front order, followed by vertices created by later layers. The merge:

1. copies `transition.transition_cells`;
2. maps each regular initial-front vertex ID to the matching `transformed_front_volume_vertex_ids` entry;
3. appends only regular vertices after the initial-front prefix;
4. remaps every regular-cell vertex through that shared-interface mapping or the appended-vertex offset;
5. appends regular cells and metadata in matching order.

This avoids duplicate coincident interface nodes and keeps transition and regular cells topologically connected. The function validates the initial-front prefix coordinates, mapping sizes, cell/metadata counts, vertex references, and `VertexId` overflow. Inputs are not mutated.

The merge is deliberately separate from both generators so the caller can inspect, write, or reject either intermediate result before producing the final mesh.

## Tests

- A skewed affected Quad chooses the diagonal whose worst triangle skewness is lower.
- A symmetric affected Quad uses the deterministic point-ID tie-break rule.
- A Quad without split branch vertices remains a Quad.
- An affected Quad becomes two triangles, produces no omitted-Quad diagnostic, and enters the tetra-only transition path.
- Debug output is absent by default and produces the two configured Legacy VTK files when enabled.
- `generateRegularLayers()` does not perform a multi-normal transition and accepts the transformed front explicitly.
- Merging offsets regular-cell connectivity and preserves metadata ordering.
- Existing mixed Triangle/Quad topology, triangle transition, and regular-layer integration tests remain passing.
