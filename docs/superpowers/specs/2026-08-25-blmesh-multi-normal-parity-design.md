# BLMesh Multi-Normal Behavioral Parity Design

## Goal

Replace the simplified multi-normal implementation with a function-by-function mechanical port of BLMesh's MNormal algorithm. For the same triangulated surface, thresholds, initial height field, and deterministic input order, the new implementation must select the same complex vertices, splitter combinations, face groups, branch directions, transition topology, and collision-resolved displacement behavior as BLMesh.

The port may rename global types and adapt ownership, errors, and public interfaces. It must not simplify, reorder, or reinterpret BLMesh's algorithm.

## Scope

The work has two sequential parts:

1. Restore parity for complex-node splitting and virtual-sphere strategy selection.
2. Restore parity for the per-vertex transition length field and surface-intersection resolution loop.

The existing explicit pipeline remains:

```text
build surface topology
    -> build initial GrowthFront
    -> generateMultiNormalTransition()
    -> generate regular layers from transformed_front
    -> merge transition and regular volume cells
```

Multi-normal processing remains a separately callable stage after surface topology construction and before regular boundary-layer generation. Debug VTK output remains disabled by default and explicitly enabled through `MultiNormalDebugOutput`.

## Porting Rule

Each selected BLMesh function is copied structurally into a focused `boundary_mesh` implementation. The following changes are allowed:

- `BLVector` becomes `Vector3` or `Point3` according to semantics.
- Raw integer point and face IDs become `VertexId`, `SurfaceFaceId`, or `std::size_t` at container boundaries.
- BLMesh macros become fields of `MultiNormalOptions` or named internal constants with the same default values.
- Singletons and mutable process-global caches become explicitly owned objects.
- Logging and exceptions become `Result<..., MultiNormalError>` where failure is recoverable.
- Include paths, namespaces, container ownership, and const-correctness may change.

The following changes are forbidden unless a parity test proves that they do not affect the result:

- changing loop or candidate-enumeration order;
- replacing splitter-subset enumeration with an all-convex-edge shortcut;
- omitting plane-ridge candidates or manifold-boundary validation;
- changing one-percent strategy-improvement semantics;
- replacing BLMesh strategy scoring with a new heuristic;
- using a single global transition height in place of the per-vertex length field;
- accepting an intersecting candidate without running BLMesh's shrink-and-retry policy.

## Complex-Node Split Port

The current `multi_normal_split_planner.cpp` computes one grouping from all detected convex ridges. It will be replaced by a mechanical port of this BLMesh decision chain:

1. Compute the original single-normal visibility and skewness.
2. Skip open/non-manifold fans and vertices below `split_skewness_threshold`.
3. In cyclic incident-edge order, compute BLMesh's signed ridge value using adjacent face normals and neighboring edge midpoints.
4. Begin at `plane_skewness_threshold`; increase the threshold by `0.01` until no more than 16 candidate splitter edges remain.
5. Enumerate splitter subsets in BLMesh `Splitter::getSpliiter()` order.
6. Classify each selected splitter as convex when its ridge value exceeds `convex_skewness_threshold`.
7. Reject a subset with no convex splitter or with more than BLMesh's allowed number of plane ridges.
8. Build the same cyclic contiguous face classification for every surviving subset.
9. Compute each group's most-normal direction with BLMesh geometry functions.
10. Construct the same virtual-sphere boundary edges, manifold triangles, coordinates, and splitter source IDs.
11. Reject invalid virtual-sphere boundaries.
12. Limit the strategy collection to `maximum_strategy_count` with BLMesh's ordering and trimming rule.
13. Run the same virtual-sphere generation, topology/point/merge optimizers, mesh evaluation, and final-strategy selection.
14. Convert the selected virtual mesh to `VertexSplitPlan` without changing branch order.

The option fields `plane_skewness_threshold` and `maximum_strategy_count` must be read by the implementation and covered by behavior tests. The BLMesh `MAX_PLAIN_RIDGE` value will be exposed as a named option only if callers need to vary it; otherwise it remains a parity constant.

## BLMesh Compatibility Components

The port will retain BLMesh's component boundaries where they carry algorithmic behavior:

- geometry primitives equivalent to `geometryfunction`;
- splitter-subset enumeration equivalent to `Splitter`;
- virtual-sphere boundary and manifold validation;
- virtual-sphere mesh storage and hashing;
- strategy generation and scoring;
- topology, point, merge/combine, and most-normal optimization used by strategy generation;
- mesh evaluation used to choose the final strategy.

These components will be private implementation details of `BoundaryMesh::BoundaryLayer`. No BLMesh global types will appear in public headers.

## Mixed Triangle/Quad Surfaces

BLMesh's MNormal core is triangle based, while `GrowthFront` supports Triangle and Quad faces. The established mixed-face rule remains:

1. Incident-face geometry is evaluated from the real mixed surface.
2. Complex vertices are split and the new topological surface is stitched.
3. After topology construction, every Quad containing a multi-normal branch vertex is split into two Triangles.
4. The selected diagonal minimizes the maximum equiangular skewness of the two Triangles.
5. A score tie within `1e-12` selects the diagonal incident to the smallest topology vertex ID.
6. The resulting Triangle faces enter the same BLMesh-compatible transition-volume path.

Unaffected Quads remain Quads on `transformed_front` and are handled later by regular-layer generation.

## Transition Length Field and Intersection Resolution

The constant displacement currently applied by `multi_normal_transition_builder.cpp` will be replaced with a per-topology-vertex length field. Its initial values come from the regular first-layer height field supplied by the caller. During the multi-normal-only stage, non-split vertices have zero displacement, matching `Generate_preVol()`.

The following BLMesh functions are ported mechanically:

- `FixedLength()` local-edge height limiting and neighboring-ratio propagation;
- `InitLengthField()`;
- `RebuildPointNeighbors()`;
- `SmoothLengthField()` with the 1.1 neighbor upper bound;
- `BuildLayerPoints()`;
- `CheckSurfaceIntersection()` and `CheckOuterSurfaceIntersection()`;
- `ShrinkLengthField()`;
- `ZeroLengthField()`;
- `ResolveLengthField()`.

Candidate resolution follows BLMesh exactly:

1. Smooth the initial length field.
2. Build a candidate transition volume.
3. test the displaced outer surface against the bottom and displaced surfaces;
4. collect every point belonging to an illegally intersecting face;
5. for at most 20 iterations, multiply affected lengths by `0.8`, smooth, and rebuild;
6. after those iterations, set affected lengths to zero, smooth, and retry once;
7. fail with a structured `MultiNormalError` if the zero-length retry still intersects.

The accepted length field is used both for transition-cell vertices and `transformed_front`, so debug volume and surface outputs describe the same geometry.

## Transition Topology

The Triangle tetrahedralization remains a direct port of `MNormalMesh::BuildCandidateVolume()`:

- bottom IDs are deduplicated by original source point;
- only split branches move during the transition stage;
- minimum lower-ID selection and `reverse_order` determine diagonals;
- repeated-bottom-ID cases preserve BLMesh's conditional extra Tetra;
- geometrically repeated four-point candidates are omitted;
- winding normalization may swap two Tetra indices only after connectivity is chosen.

No Pyramid, Prism, or Hexa is emitted by this transition builder. Affected Quad sweep regions are triangulated first and therefore use the same tetra-only path.

## Errors and Diagnostics

New structured errors distinguish:

- invalid or non-manifold local virtual-sphere boundaries;
- no valid BLMesh strategy for a selected complex vertex;
- inconsistent adapter ID mappings;
- non-finite directions or length values;
- failure to resolve transition-surface intersection after BLMesh retries.

Optional debug output includes:

- the transformed topology before displacement;
- the accepted displaced `transformed_front`;
- accepted transition Tetra cells;
- optionally, rejected candidate fronts from each collision-resolution iteration.

Default behavior produces no debug files.

## Verification

Verification is based on parity, not only validity.

### Local parity fixtures

For the same ordered incident fan, run the original BLMesh implementation and the port and compare:

- whether the center vertex is selected as complex;
- candidate ridge values and splitter IDs;
- enumerated and retained splitter subsets in order;
- cyclic face classifications;
- branch directions within `1e-12`;
- virtual-sphere validity;
- selected strategy and final branch order.

Fixtures cover convex-only, plane-plus-convex, more-than-16 initial candidates, multiple valid strategies, strategy-count trimming, invalid manifold boundaries, and no-improvement cases.

### Length-field parity fixtures

Compare every iteration's length vector, bad face IDs, bad point IDs, retry count, and accepted/rejected result for synthetic colliding and non-colliding fronts.

### Transition topology fixtures

Retain all existing `BuildCandidateVolume()` equality and ordering cases, plus affected-Quad triangulation cases.

### Real-case acceptance

Run `2dot5_cf` with multi-normal enabled and regular-layer count zero. Acceptance requires:

- debug transition volume and transformed-front VTK files are written;
- no illegal transformed-front intersection remains after resolution;
- no non-adjacent transition-cell boundary triangles intersect;
- every retained Tetra has four distinct coordinates and positive nonzero signed volume;
- the split plans and resolved per-point heights match a BLMesh reference run using the same input and options.

After this passes, run the full Release test suite and a regular-layer integration run to confirm that the accepted transformed front can seed normal layer generation and the two volume meshes merge correctly.

## Delivery Boundary

This work modifies only the existing `codex/multi-normal-topology-transition` branch. It does not link to an external absolute BLMesh source path and does not require BLMesh to be installed. Mechanically ported code lives in this repository and is built as a private part of the boundary-layer library.
