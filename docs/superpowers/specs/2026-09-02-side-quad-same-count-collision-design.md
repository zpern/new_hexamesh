# Side Quad `same_count` Collision Design

## Goal

Make layer-candidate self-collision respect complete side-Quad topology before
triangle narrow-phase testing. Identical shared sides must not be tested as two
independent triangle sets, and two distinct sides sharing an edge must not be
rejected because polygon clipping produces a floating-point-area residue on
that shared edge.

## Evidence

The existing complete-boundary metadata correctly preserves
`source_vertex_id`, `layer`, and `branch_id`. The first-layer plane failures do
not have missing topology keys:

- identical copies of one side Quad have `same_count == 4` and are already
  recognized eventually as one shared face, but only after triangle contact
  classification has run;
- every observed first-layer plane failure has `same_count == 2`, representing
  two distinct side Quads sharing one vertical edge;
- the reported coplanar overlap areas are approximately `5e-19` through
  `1.3e-17`, which are numerical residues rather than physical overlap.

The curved case has no first- or second-layer collision stop. Its later sample
intersections include non-adjacent top/side faces with no shared topology keys,
so they must remain detectable.

## Complete-Face Comparison

Before triangle narrow-phase testing, compare the complete boundary-key sets
of the two parent faces and compute `same_count`. A key matches only when all
of `source_vertex_id`, `layer`, and `branch_id` match. Ordering and triangle
diagonal do not affect the count.

The comparison also distinguishes complete side Quads from top Triangles.

## Required Rules

### `same_count == 4`

When both parent faces are side Quads and all four complete topology keys
match, they are two representations of the same logical shared side. Skip all
triangle-pair geometry tests for this face pair before narrow-phase contact
classification.

### `same_count == 2`

When both parent faces are distinct side Quads and exactly two adjacent keys
match in both Quads, they share a legitimate topology edge. Continue geometric
testing so a genuine fold or overlap can still be rejected.

For coplanar contact, measure overlap relative to the shared edge and local
face scale. Treat only floating-point-sized area confined to the shared-edge
neighborhood as legal shared-edge residue. Any overlap exceeding the
scale-derived area tolerance, or any evidence extending away from the shared
edge beyond the corresponding length tolerance, remains illegal.

The tolerances are derived from machine epsilon and the maximum squared edge
length of the two complete faces. No fixed model-unit tolerance is introduced.

### Other Values

- `same_count == 0`: retain existing non-adjacent collision behavior.
- `same_count == 1`: retain vertex-contact behavior and detect penetration.
- non-adjacent curved-case top/side intersections remain illegal.
- complete top-face identity retains its existing legal shared-face behavior.

## Implementation Shape

Add a small complete-boundary relation helper in the triangle-contact module.
It returns face kinds, `same_count`, adjacency of the two shared keys, and the
local geometric scale. `hasIllegalTriangleContact()` consults this relation
before calling the triangle contact classifier:

1. identical side Quad: return legal immediately;
2. otherwise classify triangle contact;
3. shared side-Quad edge: apply the scale-aware shared-edge residue rule;
4. all other relations: use the existing legality rules.

This change is confined to collision contact classification. It does not
change CGNS welding, sliding projection, growth directions, termination
propagation, or boundary output.

## Tests

Tests are written and observed failing before production changes. They cover:

1. identical side Quads with the same four keys, independent of order and
   triangle diagonal, are legal without triangle narrow-phase dependence;
2. branch or layer differences prevent four-key identity;
3. distinct side Quads sharing exactly one edge accept a floating-point-area
   coplanar residue modeled after the plane failure;
4. the same shared-edge topology with a clear coplanar fold/overlap is illegal;
5. non-adjacent intersecting faces remain illegal;
6. existing triangle, shared-vertex, shared-edge, and shared-face contact tests
   remain passing.

## Real-Case Verification

Build Release and run the full configured test suite. Then run `plane.cgns` and
`curve.cgns` with first height `0.1`, growth ratio `1.2`, 20 requested layers,
maximum skewness `1`, and isotropic height `1`.

For plane, the six first-layer stops caused only by `1e-18`-scale shared-edge
residue must disappear. For curve, genuine later non-adjacent collision stops
must not be globally suppressed. Compare per-layer accepted cell counts and
final stop-reason totals against the recorded baseline.

## Success Criteria

- Side-Quad identity is decided before triangle narrow phase.
- Numerical shared-edge residue is legal at any model scale.
- Genuine adjacent folding and non-adjacent intersection remain illegal.
- Targeted tests, full tests, Release build, and both real cases verify the
  intended behavior.
