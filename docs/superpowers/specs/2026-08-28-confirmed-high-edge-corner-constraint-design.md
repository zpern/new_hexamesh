# Confirmed High-Edge Corner Constraint Design

## Purpose

Prevent a stopped Triangle or Quad from selecting a temporary same-layer
neighbor as its high neighbor. The corner-layer constraint must be based on the
neighbor that actually survives into the next layer.

This fixes the reproduced `2dot5_cf` case in which source Quad `19357` stops at
18 trial layers. Its edge-0 neighbor is still present in layer 18 but also stops
at 18, while its edge-1 neighbor reaches layer 19. The current implementation
selects edge 0 too early, so a face incident to an edge-1 non-contact corner is
incorrectly allowed to reach layer 19.

## Required Invariant

Let source face `F` have completed trial layer `k`, and let `H` be its first
edge neighbor, in local-edge order, that is accepted into trial layer `k + 1`.

- If no such neighbor exists, `F` has no high edge and introduces no corner
  constraint.
- If `F` is a Triangle, every patch face incident to the vertex opposite the
  selected high edge, except `F` and `H`, must have at most `k` trial layers.
- If `F` is a Quad, every patch face incident to either vertex outside the
  selected high edge, except `F` and `H`, must have at most `k` trial layers.
- The first confirmed high edge is retained. Other prospective high-edge
  neighbors are constrained back to at most `k` through the same non-contact
  corner rule.

The resulting transition state therefore has at most one high edge for both
Triangles and Quads.

## Current Failure

`TerminationPropagator::filterSingleHighEdgeCandidates()` currently processes
a stop cell immediately when that cell is still present in layer `k`. It scans
the layer-`k` candidates and selects the first edge neighbor that is present and
whose allowed limit is above `k`.

Presence in layer `k` does not prove that the neighbor will be accepted into
layer `k + 1`. A neighbor selected by this test can later stop at the same
height as `F`. Final transition coordination then computes a different high
edge from accepted layer counts, after the corner constraint has already been
applied to the wrong vertices.

## Design

### Stop-cell states

The existing `StopCell` states are retained:

- `candidate_present == false`: `F` disappeared before the current candidate
  layer. Its completed layer is `step.layer - 1`, so the current candidate
  layer already reveals its possible high neighbors. Process it immediately.
- `candidate_present == true`: `F` is accepted in the current layer but must
  stop at that layer. Do not select a high neighbor yet. Add `F` to
  `pending_stop_cells` and defer processing until the next layer.

At the next call, a pending face is represented as a non-present stop cell with
`completed_layer = step.layer - 1`. Any neighbor remaining in
`output.next_front` has produced a candidate one layer above `F`, so it is a
real prospective high neighbor.

### Confirmed-edge selection

For a non-present stop cell:

1. Scan `entry.edge_rules` in deterministic local-edge order.
2. Select the first rule whose neighbor is present in
   `output.next_front.source_face_ids`.
3. If no rule qualifies, make no corner-layer change.
4. Limit every face in the selected rule's
   `non_contact_corner_faces` to `completed_layer`.
5. Preserve `DirectStop` as the stronger limit kind; otherwise mark a lowered
   face as `NeighborConstraint`.
6. Run the existing edge-neighbor `propagate()` operation.
7. Run `filterCandidates()` before committing the current layer.

The existing precomputed `EdgeRule::non_contact_corner_faces` supports both
Triangles and Quads and requires no topology representation change.

### Pending scheduling

When a present stop cell is deferred, it must be inserted into
`pending_stop_cells` even if no constraint changed during the current call.
The list remains sorted and unique.

Pending faces must not be treated as ordinary continuing faces. They exist only
as deferred topology decisions and are processed independently of
`current_front.source_face_ids` on the next call.

### Final validation

`TransitionLayerCoordinator::coordinate()` must validate the same invariant
from final `occupied_layers`:

- identify the final high edge of each low face;
- reject more than one final high edge;
- for the selected high edge, inspect the source face's non-contact vertices;
- reject any patch face incident to those vertices whose trial-layer count is
  above the low face's trial-layer count.

The growth stage is responsible for satisfying the invariant. Final validation
is defense in depth and must return a coordination error rather than silently
emitting a non-conforming template.

Because final coordination currently receives only trial counts and topology,
the check will compare trial-layer counts. This matches the growth constraint
and correctly handles the reserved-layer conversion.

## Determinism

When multiple edge neighbors survive into the next candidate layer, the first
edge in the source face's local-edge order is retained. The choice is
independent of source-face ID ordering and container iteration order.

## Error Handling

- Missing constraints or topology entries continue to return
  `InvalidFaceConstraintState`.
- A final corner-layer violation returns a dedicated transition-coordination
  error containing the low source face, selected high edge, non-contact vertex,
  and violating source face.
- No partially constructed transition cells are committed after a validation
  failure.

## Tests

### Unit regression: delayed selection drift

Construct a Quad `F` with ordered edge neighbors such that:

- `F` is accepted through layer `k` and then stops;
- the edge-0 neighbor is accepted through `k` but not `k + 1`;
- the edge-1 neighbor is accepted into `k + 1`;
- an additional face incident only to an edge-1 non-contact corner attempts
  layer `k + 1`.

Verify that processing layer `k` only schedules `F`, processing layer `k + 1`
selects edge 1, and the additional corner face is filtered back to `k`.

### Triangle regression

Repeat the timing pattern for a Triangle and verify that every face incident to
the opposite vertex is limited to `k`.

### Multiple prospective high neighbors

Allow two neighbors to enter layer `k + 1`. Verify that local-edge order retains
the first and the non-contact-corner rule filters the other back to `k`.

### No high neighbor

Stop all neighbors at `k`. Verify that deferred processing makes no corner
constraint and leaves no pending entry.

### Final validator

Pass manually constructed final counts containing a corner-only violation and
verify that transition coordination rejects them. Verify valid Triangle and
Quad single-high-edge configurations still succeed.

### Real-case verification

Regenerate `2dot5_cf` with:

```text
first height       0.1
growth ratio       1.2
requested layers   20
maximum skewness   1.0
isotropic height   1.0
multi-normal       disabled
```

Verify source face `19357` either selects a high edge whose non-contact corner
faces have no more than 18 trial layers, or is reduced to a configuration with
no high edge. In particular, source face `19358` must not remain at 19 while it
is incident to a non-contact corner of a high edge on face `19357`.

## Scope

This change modifies only single-high-edge stop coordination and its final
validation. It does not change growth geometry, collision evaluation, quality
thresholds, transition templates, multi-normal topology, or the requested
neighbor-layer difference.
