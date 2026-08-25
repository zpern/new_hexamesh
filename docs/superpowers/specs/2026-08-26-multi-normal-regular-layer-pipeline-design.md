# Multi-Normal and Regular-Layer Pipeline Design

## Goal

Add a reusable boundary-layer generation entry point that applies the existing
multi-normal topology transition to Wall faces before regular-layer growth.
The transformed multi-normal surface is the regular generator's layer zero.
The transition cells and regular-layer cells are merged only after regular
growth finishes.

The production CGNS path reads the complete boundary surface, but only faces
tagged `Wall` participate in multi-normal processing and regular-layer growth.
The original non-Wall farfield and the final exposed boundary-layer top are
preserved as output surfaces.

## Public Boundary

Introduce `generateBoundaryLayers()` as the production orchestration API.
Keep `generateRegularLayers()` unchanged as the lower-level API that grows
only regular layers from a supplied layer-zero front. This preserves existing
call semantics and makes the two stages independently testable.

The unified input contains the complete surface and topology, the Wall growth
patch and initial Wall front, source-vertex growth profiles, multi-normal
options, and regular-layer options. The result contains:

- the merged transition and regular-layer volume mesh;
- the multi-normal transition result for diagnostics;
- regular-layer growth records;
- the original non-Wall farfield plus the final exposed layer top;
- the final exposed boundary-layer top as a separate surface.

Multi-normal debug VTK output remains disabled by default and continues to be
controlled by `MultiNormalDebugOutput`.

## Processing Sequence

1. The CGNS reader loads the complete tagged surface.
2. `GrowthPatchBuilder` selects only `Wall` faces.
3. `GrowthFrontBuilder` constructs the initial Wall front.
4. `generateBoundaryLayers()` calls `generateMultiNormalTransition()` on that
   Wall front.
5. The returned `transformed_front` becomes regular layer zero.
6. All front-local state derived from connectivity is rebuilt from the
   transformed front: vertex-to-face and edge-to-face adjacency, smoothed
   growth directions, collision primitives, active-face state, and exposed
   boundary tracking. No adjacency from the original Wall front is reused.
7. Regular growth generates the requested number of regular layers.
8. `mergeMultiNormalAndRegularMeshes()` joins the transition and regular cells
   by the explicitly recorded transition/front vertex mapping.
9. The output builders emit the merged volume, the original non-Wall farfield,
   and the final exposed boundary-layer top.

The transition is not counted as a regular layer. With a requested layer count
of five, the transformed front is layer zero and five regular layers are
attempted above it.

## Vertex Identity and Profile Inheritance

A front vertex is identified by `(source_vertex_id, branch_id)` within a
layer. Ordinary vertices use branch zero. Multi-normal split branches retain
distinct branch identifiers throughout regular growth, collision handling,
layer-vertex records, exposed-boundary tracking, top-surface construction, and
farfield vertex deduplication.

All branches derived from one source vertex inherit the same
`SourceVertexGrowthProfile`. A missing source profile is an input error. Two
branches may occupy the same source position at layer zero, but must never be
merged solely because their `source_vertex_id` and layer match.

The transformed-front vertex order is retained as the leading vertex range of
the regular volume mesh. This is required by the existing mesh merge contract
and prevents duplicate interface vertices.

## Surface Outputs

The final boundary-layer top is built from the regular generator's real
exposed boundary, including faces that stopped before the requested maximum
layer. Face orientation follows the existing outward-interface convention.

The combined farfield surface contains:

- every original face tagged `Farfield`, unchanged geometrically; and
- the final exposed boundary-layer interface.

It excludes the original Wall faces after a transition or regular cell covers
them. For a zero-regular-layer run, the multi-normal transformed front is the
exposed boundary-layer top when a transition was applied; otherwise the
original Wall front is the top.

Surface vertex deduplication includes `branch_id` in addition to source vertex
and layer, so distinct multi-normal branches remain topologically distinct.

## Failure Handling

The unified API returns a stage-qualified error for multi-normal generation,
regular-layer generation, or mesh merging. Invalid transformed-front mappings,
missing source profiles, and inconsistent branch identities fail before cells
are committed. Debug output failure does not silently replace a generation
error.

The production CLI reports generation failure without writing partial final
outputs. Optional multi-normal debug files retain their existing behavior.

## Verification

Tests are added before implementation and cover:

- multi-normal-disabled compatibility with the existing regular generator;
- transformed-front vertices becoming the exact regular layer-zero prefix;
- split branches inheriting profiles while remaining distinct;
- no duplicate vertices across the transition/regular interface;
- correct merged cell roles and metadata;
- zero-layer top-surface behavior;
- five regular layers above a multi-normal transition;
- farfield and final-top output topology;
- CLI routing of only Wall faces into the growth stages.

After the complete test suite passes, run
`2dot5_cf.cgns` with its boundary-condition mapping (`Far: 1,2`, `Wall: 3,4`),
multi-normal enabled, and regular layer count five. The acceptance artifacts
are the merged volume VTK, farfield-boundary VTK, final boundary-layer top VTK,
and optional transition/front debug VTK files. Report generated cell counts,
layer acceptance/stopping counts, and any collision step reductions or invalid
cells.
