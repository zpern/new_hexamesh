# Batched External Distance Search Design

## Goal

Remove the per-owner global provisional rebuild loop used to resolve colliding
external transition patches while preserving independent distance scales and
the existing collision/keep-hexa decisions.

## Selected Approach

All initially colliding external owners participate in a parallel search. Each
owner keeps independent `low`, `high`, probe, and bracket state. One search
round writes every active owner's next scale, builds one global provisional
transition, calls `TransitionBoundaryChecker::inspect()` once, and updates all
owner states from the returned `ExternalPatch` owner IDs.

The halving phase runs until every owner is bracketed or reaches the minimum
scale. Owners that cannot become safe retain the existing behavior: dependent
high faces force the outer rollback path; otherwise the owner is converted to
KeepHexa. The 12 binary-search rounds then operate on all bracketed owners in
parallel. A final global build uses every owner's accepted low scale.

Safe external patches identified by the initial scan remain unchanged and do
not enter the search.

The collision broad phase is retained across search rounds. The initially
assembled boundary is grouped by external `source_face_id` in an
`IncrementalCollisionIndex`; non-external triangles share an immutable base
group. After a trial geometry build, each active external group is erased and
reinserted, and only its replacement triangles are queried against the index
and fixed obstacle sources. This avoids rebuilding and rescanning unaffected
collision geometry.

## Alternatives

- Per-owner search is behaviorally simple but causes hundreds of global builds
  on the anisotropic case.
- Local patch-only rebuilds could be faster but require a new local topology
  and collision assembly API and carry higher correctness risk.
- Parallel global search is selected because it changes scheduling, not the
  transition representation or exact collision predicates.

## Correctness

Collision membership is keyed by `(ExternalPatch, source_face_id)`. Each owner
updates only from its own collision status. Output scales are deterministic for
sorted owner IDs. Single-owner behavior remains equivalent to the existing
search. Multi-owner tests cover different collision thresholds and assert that
global build count is bounded by the number of search rounds rather than the
number of owners.

## Validation

Run resolver, transition, coordination, and collision tests. Then rerun the
anisotropic case with first height `0.1`, growth ratio `1.2`, and 20 layers.
Compare the first-layer cell count and confirm the second layer completes
without per-owner 20–30 second stalls.
