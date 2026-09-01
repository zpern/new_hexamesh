# Iterative Sliding Normal Constraint Design

## Goal

Match BLMesh normal smoothing for Symmetry and Internal vertices by keeping
their directions on the applicable sliding-surface tangent space before and
after every synchronous smoothing round. Retain final candidate-position
projection as a separate geometric safeguard.

## Confirmed Difference

BLMesh constrains a sliding node's initial normal and constrains the newly
smoothed normal every time a smoothing iteration is applied. The constrained
normal becomes the input to the next iteration.

The current pipeline smooths unconstrained directions for as many as 100
rounds and calls `SlidingConstraints::constrainDirection()` only after the
entire smoothing operation. On the curve case, this final correction averages
about 33.6 degrees on layer one and 12 degrees on layer three. Layer three is
also the first layer with locally inverted candidates and subsequent real
intersections. The plane correction is much smaller.

## Required Order

For each layer:

1. Build `SlidingConstraints` before growth-field smoothing.
2. Constrain every raw sliding-vertex direction at its current front position.
3. Initialize smoothing from those constrained directions.
4. In each synchronous smoothing round, calculate all tentative directions
   from the same current buffer.
5. Constrain every tentative sliding direction before swapping it into the
   current buffer.
6. Use the constrained current buffer for the next round, visibility checks,
   predicted-position height smoothing, and skewness refinement.
7. Constrain directions returned by skewness refinement before returning the
   final smoothed field.
8. Keep the existing final direction constraint in `RegularLayerStepper` as a
   defensive idempotent check.
9. Keep final candidate-position projection unchanged.

Axis-aligned planes, curved surfaces, multiple compatible sliding regions, and
vertices without sliding regions use the same orchestration. A constraint
failure remains a typed growth-direction failure; it must not silently fall
back to the unconstrained direction.

## Component Boundary

`RegularLayerStepper` continues to build `SlidingConstraints`, because it owns
the per-layer front and sliding-surface set. `GrowthFieldSmoother` accepts an
optional immutable constraint provider and applies it at the synchronization
boundaries described above. The smoother does not rebuild surfaces or infer
boundary regions.

The existing no-constraint smoother API behavior remains available to unit
tests and non-sliding callers through a null/default provider. No changes are
made to CGNS input, collision classification, termination propagation, or
boundary output.

## Tests

Tests are written and observed failing before production changes:

- a curved sliding vertex begins and remains tangent during iterative
  smoothing, not only after the final return;
- a neighbor receives influence from the constrained direction rather than an
  unconstrained intermediate direction;
- an axis-aligned sliding plane remains tangent through multiple rounds;
- a non-sliding front produces the same result as the existing API;
- constraint failure propagates as a typed smoother/stepper failure;
- final candidate positions remain on every referenced sliding surface.

Integration tests retain existing plane, curve, multi-region, and final
projection coverage.

## Real-Case Verification

Build and run all Debug and Release tests. Run plane and curve with first
height `0.1`, growth ratio `1.2`, 20 requested layers, maximum skewness `1`,
and isotropic height `1`.

The plane first layer must remain complete at 11302 cells. The curve first two
layers must remain complete. Compare curve locally inverted, collision, and
neighbor-constraint counts with the current baseline of 69, 199, and 4235,
respectively. Any reduction must come from changed constrained geometry, not
from suppressing collision checks.

## Success Criteria

- Sliding directions are constrained before and after every smoothing round.
- Constrained directions, not discarded unconstrained intermediates, drive
  subsequent smoothing and height prediction.
- Final position projection remains active.
- Full tests pass in Debug and Release.
- Plane remains fixed and curve real-case diagnostics demonstrate the effect.
