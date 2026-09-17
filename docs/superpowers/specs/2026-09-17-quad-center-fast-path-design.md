# Quad Top-Cap Center Fast Path

## Goal

Reduce terminal-layer construction time without changing the validity rules or
the fallback result for difficult quad columns.

## Design

`findPositiveQuadTopCapCenter` already evaluates the arithmetic mean of the
four bottom and four top vertices before its exhaustive search. When that mean
passes `quadTopCapValidityMargin` and its margin is greater than the caller's
`volume_tolerance`, return it immediately. Otherwise retain the existing
12-constraint, four-active-constraint search unchanged.

The fast path therefore uses the same Pyramid/Tetra evaluators as the fallback;
it does not admit a center that the current implementation considers invalid.

## Verification

- A regular hexahedral column must return its exact arithmetic mean.
- A warped fixture whose arithmetic mean is invalid must still find the same
  valid fallback center.
- Transition template and pipeline tests must remain green.
- The 2dot5 20-layer Release run is the performance acceptance case.
