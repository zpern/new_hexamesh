# Verdict-Aligned Equiangle Skew Design

## Goal

Make the internal face-based equiangle-skew calculation detect reflex corners
and severe invalid geometry consistently with the metric family used by
ParaView's Verdict-backed `Equiangle Skew` quality measure. This correction is
a prerequisite for strengthening skewness-aware normal smoothing because the
optimizer and the final quality gate must respond to the same bad geometry
that ParaView reports.

This work remains isolated on the
`codex/skewness-aware-normal-smoothing` branch.

## Current Problem

`face_skewness.cpp` currently obtains every corner angle with `acos` of the two
incident edge directions. `acos` returns only an unsigned angle in `[0, pi]`,
so a concave quadrilateral's reflex interior angle is incorrectly represented
by its smaller complementary orientation. The function then clamps the final
skewness to `[0, 1]`, hiding any value that should exceed one for a reflex or
severely invalid face.

The volume evaluators take the maximum face skewness. Consequently, a missed
reflex corner can make an invalid upper or lower face appear acceptable and
can give the normal refiner a misleading objective.

## Selected Approach

Implement a local, dependency-free, Verdict-aligned oriented-corner
calculation inside the existing surface-quality component. Do not add a
runtime or build dependency on Verdict, and do not add skewness arrays to VTK
output.

The implementation will be validated against analytic reference cases and
reference values obtained from ParaView/Verdict. Keeping the calculation
local preserves the low fixed cost required by the direction refiner, which
evaluates many trial cells.

## Oriented Face Angles

For each ordered triangle or quadrilateral:

1. Validate that all coordinates and incident edge lengths are finite and
   non-degenerate using the existing tolerance behavior.
2. Construct a stable reference normal from the ordered polygon. The normal
   must reverse when the vertex order reverses and must fail explicitly when
   the polygon has no numerically stable orientation.
3. At each vertex, combine the edge dot product with the signed cross-product
   component along the reference normal to determine the oriented interior
   angle.
4. Represent convex corners in `(0, pi]` and reflex corners in `(pi, 2*pi)`.
5. Use the minimum and maximum oriented interior angles in the existing
   equiangle-skew formula.

For a warped quadrilateral, all corners use the same stable ordered-polygon
reference orientation. This makes reflex classification deterministic and
prevents independent corner normals from changing the meaning of the angle.
If the ordered polygon is too degenerate to establish that orientation, face
evaluation returns an error rather than manufacturing a finite skewness.

Reversing the complete vertex order must preserve skewness: both the reference
normal and edge traversal reverse together, leaving the geometric interior
angles unchanged.

## Skewness Range and Quality Gate

The final skewness result will no longer be clamped to `[0, 1]`.

- Ordinary valid faces continue to produce values in `[0, 1]`.
- A reflex corner or severe invalid configuration may produce a value greater
  than one.
- Non-finite results remain evaluation errors.

`VolumeCellQualityOptions::maximum_skewness` remains restricted to `[0, 1]`.
Therefore a finite internal skewness above one naturally fails the existing
quality gate without requiring a new stop reason or configuration range.

The prism and hexahedron evaluators continue to define cell skewness as the
maximum skewness of all constituent faces. Existing validity, signed-volume,
local-inversion, collision, and stopping logic remains in place.

## Interfaces and Output

The public triangle and quadrilateral skewness function signatures remain
unchanged. The prism and hexahedron evaluation result types also remain
unchanged.

No `EquiangleSkew` cell-data array or other quality field is written to VTK.
Comparison with ParaView uses controlled reference geometries and aggregate
maximum values from the existing diagnostics or benchmark output.

## Error Handling

Existing errors for non-finite coordinates and degenerate edges are retained.
A polygon whose reference orientation cannot be established reliably is
reported through the existing face-evaluation failure path. Candidate cells
containing such a face are rejected by the existing volume-evaluation and
normal-refinement fallback behavior.

The baseline direction remains available during normal refinement, so a trial
that exposes a degenerate or invalid face cannot cause smoothing itself to
fail fatally.

## Testing

Unit tests will cover:

- an equilateral triangle and a square producing zero skewness;
- known convex triangle and quadrilateral reference values;
- a planar concave quadrilateral whose reflex angle is detected and whose
  skewness exceeds one;
- the same face with reversed vertex order producing the same result;
- a warped quadrilateral with a stable orientation producing a deterministic
  reference value;
- a polygon with no stable orientation returning an evaluation error;
- prism and hexahedron cells propagating the maximum corrected face skewness;
- a cell with skewness above one failing a threshold of `1.0`;
- translation and positive uniform scaling invariance.

Reference fixtures will be checked against ParaView/Verdict's `Equiangle
Skew` values to a documented floating-point tolerance. The test suite will not
depend on ParaView at runtime.

After unit and regression tests pass, rebuild the true optimized Release and
rerun the `2dot5_cf` 20-layer case with first height `0.1`, growth ratio `1.2`,
and maximum skewness `1.0`. Compare the reported aggregate maximum with the
ParaView result and inspect any material disagreement before changing the
normal-search objective.

## Performance

The corrected calculation remains fixed-size and allocation-free. It adds a
small number of cross products and signed-angle operations per face corner.
This should be minor relative to candidate-cell construction and the bounded
direction search, but the representative Release benchmark will record total
runtime so regressions are visible.

## Scope Boundaries

This phase corrects and validates the quality metric only. It does not:

- write new VTK fields;
- change layer heights or growth profiles;
- change the `0.8` normal-refinement activation threshold;
- add the requested stronger behavior above `0.9`;
- prioritize upper faces over side or lower faces;
- change search angles, azimuth counts, or iteration counts.

The stronger upper-surface objective for cells above `0.9` will receive a
separate design after this metric matches the expected ParaView/Verdict
behavior.

## Acceptance Criteria

This phase is complete when:

- analytic convex and concave test cases produce their expected values;
- reversed face ordering preserves skewness;
- reflex geometry is no longer hidden by an unsigned angle or `[0, 1]` clamp;
- prism and hexahedron quality gates reject finite skewness above one;
- all existing and new tests pass;
- a true Release `2dot5_cf` 20-layer benchmark is reported;
- the aggregate result is consistent with ParaView/Verdict, or any remaining
  difference is isolated and explained before optimizer work resumes.

## Verification Results (2026-08-25)

The implementation was verified in the isolated feature worktree at commit
`3a0ada1`.

- The true MSVC Release compile command contains `/O2`, `/Ob2`, and `NDEBUG`.
- The CGNS Release CLI size is 3,329,024 bytes.
- The non-CGNS Release build succeeded and all 44 registered tests passed.
- New tests cover a planar reflex quadrilateral with an analytic expected
  value, reversed ordering, a stable warped quadrilateral, translation and
  positive uniform scaling, cancelling polygon orientation, and hexahedron
  propagation above one.
- Existing locally inverted prism/hexahedron diagnostics remain available:
  a face with a cancelling area vector is treated as degenerate quality by
  the volume aggregator without replacing the fixed-subtet validity class.

The `2dot5_cf` A/B benchmark used first height `0.1`, growth ratio `1.2`, 20
layers, maximum skewness `1.0`, and maximum neighbor-layer difference `1`.
Its results were:

| Measurement | Disabled baseline | Enabled refinement |
| --- | ---: | ---: |
| Growth time | 292.821 s | 323.350 s |
| Skewness stops | 6 | 0 |
| Final volume cells | not written by benchmark | 917,031 |

Enabled-refinement diagnostics were:

- activated vertex-layer instances: 27,472;
- updated directions: 1,937;
- maximum candidate skewness before refinement: `1.0533`;
- maximum candidate skewness after refinement: `0.991509`;
- locally inverted stops: 15;
- collision stops: 50;
- neighbor-layer-constraint stops: 29,339;
- peak working set: 660,000,768 bytes.

The corrected metric therefore exposes a finite value above one that the old
clamp hid, and the existing direction refinement reduces that observed
candidate maximum below one. The generated mesh is:

`build-cgns/2dot5_verdict_skew_layer20/2dot5_cf_boundary_layer.vtk`

No ParaView command-line runtime was found on this machine, so the final
aggregate comparison against ParaView/Verdict remains a manual review step.
Agreement is not claimed until that value is measured. No skewness cell-data
array was added to the VTK output.
