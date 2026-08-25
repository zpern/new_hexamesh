# Split Quad Multi-Normal Transition Design

## Goal

After multi-normal vertex splitting, triangulate every transformed Quad that contains at least one split branch vertex. Choose the diagonal that minimizes the worst triangle equiangular skewness, then feed the resulting triangles through the existing BLMesh-style tetra-only transition builder.

## Scope

- Keep a transformed Quad unchanged when none of its corners is a multi-normal branch vertex.
- Triangulate a transformed Quad when at least one corner is a multi-normal branch vertex.
- Stop emitting `OmittedQuadTransition` diagnostics for affected Quads because they will no longer be omitted.
- Preserve the existing triangle transition decomposition, point-ID ordering, signed-volume correction, and degenerate-tetra filtering.
- Do not add Pyramid, Prism, or Hexa transition cells.

## Processing Order

1. Build split branch vertices.
2. Remap original Triangle and Quad faces to their branch vertices.
3. Add stitching triangles between adjacent branches.
4. Traverse the resulting front and triangulate affected Quads.
5. Move only multi-normal branch vertices by `transition_height`.
6. Build transition volume cells from all transformed Triangle faces through the existing `addTet()` paths.

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

## Tests

- A skewed affected Quad chooses the diagonal whose worst triangle skewness is lower.
- A symmetric affected Quad uses the deterministic point-ID tie-break rule.
- A Quad without split branch vertices remains a Quad.
- An affected Quad becomes two triangles, produces no omitted-Quad diagnostic, and enters the tetra-only transition path.
- Existing mixed Triangle/Quad topology, triangle transition, and regular-layer integration tests remain passing.
