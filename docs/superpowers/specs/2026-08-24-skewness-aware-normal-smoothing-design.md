# Skewness-Aware Normal Smoothing Design

## Goal

Improve boundary-layer candidate-cell skewness while smoothing growth normals.
The optimization changes growth directions only. It must preserve the existing
height-smoothing calculation, direction-safety constraints, final cell-quality
filtering, and collision handling.

The feature is initially developed and evaluated on the
`codex/skewness-aware-normal-smoothing` branch.

## Current Behavior

`GrowthFieldSmoother::smooth()` first smooths growth directions and then derives
per-vertex actual heights. `RegularLayerStepper` uses those fields to construct
candidate prism and hexahedron cells. Only after construction does it evaluate
cell validity and maximum face equiangular skewness.

Consequently, the current normal smoother receives no skewness feedback. A
direction that is smooth and visible can still create a poor candidate cell,
which is then stopped by the final quality filter.

## Selected Approach

Use a bounded, derivative-free local search after the existing direction and
height calculations:

1. Run the existing normal smoothing algorithm to obtain baseline directions.
2. Run the existing height-smoothing algorithm and freeze the resulting
   `actual_heights`.
3. Refine directions near the baseline directions using candidate-cell
   skewness as an optimization objective.
4. Return the refined directions with the unchanged heights.

This is a soft optimization. It does not require every optimized cell to meet
`maximum_skewness`. The existing final cell-quality evaluation remains the
hard acceptance gate.

## Local Objective

For a vertex, the local neighborhood consists of every front face incident to
that vertex and therefore every candidate prism or hexahedron built from those
faces.

Candidates are compared lexicographically by:

1. lowest maximum skewness among incident candidate cells;
2. lowest average skewness among those cells;
3. smallest angular deviation from the baseline smoothed direction.

The first criterion is a minimax objective. It prevents an improvement across
several ordinary cells from hiding a severe regression in one cell.

The baseline direction is always included as a candidate. A new direction is
accepted only when it produces a meaningful reduction in the local maximum
skewness. Otherwise the baseline direction is retained.

## Candidate Search

Only vertices whose incident-cell maximum skewness exceeds `0.8` are active.
This trigger controls optimization work; it does not replace or modify
`maximum_skewness`.

For each active vertex:

- build a stable orthonormal basis for the tangent plane of its current
  direction;
- sample six uniformly spaced azimuths around that direction;
- use a maximum angular offset of 5 degrees in the first search level;
- center the second level on the first-level winner and sample six azimuths at
  a 2 degree offset;
- include the current direction at every level.

All vertices read the same round-start direction buffer and write into a
separate result buffer. Updates are therefore synchronous and independent of
vertex traversal order.

The implementation runs at most two search levels. Vertices with no meaningful
first-level improvement do not enter the second level.

## Candidate Geometry and Quality Evaluation

During a vertex trial, only that vertex's candidate upper-layer position is
changed:

```text
trial_position = front_position + fixed_actual_height * trial_direction
```

Other vertices use the direction buffer from the start of the current search
level and their fixed actual heights. Each incident front face is assembled as
the same `PrismPoints` or `HexaPoints` layout used by `RegularLayerStepper` and
evaluated with the existing `evaluatePrism()` or `evaluateHexa()` functions.

Quality evaluation is local and fixed-size. Results that do not depend on the
trial vertex should be cached within a search level where practical.

## Safety Constraints and Fallbacks

A trial direction is eligible only when it:

- is finite and normalizable;
- satisfies the existing incident-face visibility requirement;
- satisfies the existing maximum-deviation constraint relative to the raw or
  baseline direction, as applicable;
- creates only valid incident candidate cells.

A trial is rejected if any incident cell is degenerate, reversed, locally
inverted, or produces a non-finite/intermediate evaluation failure. These
conditions are local candidate rejection, not a fatal smoothing error, because
the unchanged baseline remains available.

If basis construction, candidate construction, or all trial evaluations fail,
the vertex keeps its baseline direction. Existing fatal input-validation
behavior remains unchanged.

## Configuration

Add a focused skewness-normal-optimization configuration to the regular-layer
growth options. Its initial defaults are:

- enabled: `true` on the test branch;
- activation skewness: `0.8`;
- first-level angular offset: 5 degrees;
- second-level angular offset: 2 degrees;
- azimuth samples per level: 6;
- maximum search levels: 2.

The feature can be disabled to reproduce the prior smoothing path for A/B
comparison. The public option names and validation rules will follow the
project's existing option conventions. Invalid non-finite values, angular
ranges, trigger ranges outside `[0, 1]`, or zero sample/level counts are
rejected consistently with other growth-option validation.

## Diagnostics

Expose enough per-step or aggregate diagnostics to compare behavior without
changing the mesh algorithm:

- number of vertices activated by the `0.8` trigger;
- number of vertices whose direction was updated;
- maximum candidate-cell skewness before refinement;
- maximum candidate-cell skewness after refinement.

Benchmark reporting should compare optimization disabled and enabled using:

- maximum skewness;
- number of faces stopped by the skewness limit;
- total layer-generation time.

## Performance Bound

An unconditional two-level search would multiply fixed-size quality
evaluations substantially. The design limits cost through the `0.8` activation
trigger, early exit after a non-improving first level, local incident-face
evaluation, synchronous bounded rounds, and local caching.

For meshes where roughly 10--20% of vertices activate, the intended practical
overhead is approximately 20--100%. A globally poor front may cost several
times the baseline runtime; the fixed candidate and level counts prevent
unbounded iteration.

## Code Boundaries

The implementation should keep responsibilities separable:

- the existing normal smoother produces the baseline direction field;
- the existing height smoother produces and freezes `actual_heights`;
- a dedicated internal skewness-direction refiner builds trials, evaluates
  local objectives, and returns refined directions;
- `RegularLayerStepper` retains ownership of final quality acceptance and
  stopping decisions.

The refiner may reuse small candidate-cell assembly helpers shared with the
stepper, but the final acceptance path must not be duplicated or moved into the
optimizer.

## Testing

Unit tests must cover:

- incident skewness at or below `0.8` leaves the direction unchanged;
- incident skewness above `0.8` activates the search and lowers the local
  maximum when an eligible improving direction exists;
- no improving direction produces an exact fallback to the baseline;
- a direction creating a degenerate, reversed, or locally inverted cell is
  rejected;
- visibility and maximum-deviation constraints reject otherwise attractive
  candidates;
- equivalent vertex/face permutations produce correspondingly equivalent
  results;
- disabling the feature reproduces baseline directions and heights;
- enabling the feature does not change the height values produced from the
  baseline smoothing pass.

Regression tests must confirm that existing final skewness filtering,
cell-validity filtering, collision handling, and stop-reason accounting remain
unchanged.

A representative-mesh benchmark must record the disabled/enabled quality and
runtime diagnostics before this behavior is considered suitable for the main
branch.

## Acceptance Criteria

The branch is ready for evaluation when:

- all existing and new unit tests pass;
- optimization never increases an accepted vertex's local maximum skewness;
- height results match the disabled baseline for the same smoothing input;
- results are deterministic under traversal-order changes;
- representative meshes show the maximum skewness and/or skewness-stop count
  improving without violating existing validity, visibility, or collision
  rules;
- measured runtime is reported so the quality/performance tradeoff can be
  reviewed before integration.
