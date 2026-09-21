# Incremental Transition Collision State Design

## Goal

Replace repeated full exposed-boundary assembly and collision scans between
`LayerTransitionResolver` rollback iterations with a transactional incremental
state. Preserve exactly the same exposed triangles, colliding owners, rollback
IDs, transition topology, and final cell counts as
`TransitionBoundaryChecker::inspect()`.

The accepted Benchmark remains 20 layers, first height `0.1`, growth ratio
`1.2`, maximum skewness `1.0`, and 215,050 cells on layer 20.

## Current Cost

Every resolver rollback iteration currently performs a complete scan:

1. create a `TriangleKey` for every candidate triangle;
2. sort all candidate triangles;
3. cancel equal keys by odd/even multiplicity;
4. query every exposed triangle against original, historical, prior-transition,
   and sliding boundaries;
5. build a self-collision index from all exposed triangles;
6. query all exposed triangles against that index.

Benchmark layers 18 through 20 contain approximately 390,000 boundary
triangles. Each full scan costs 6.8 to 8.9 seconds even when the preceding
rollback changes only hundreds of approximately 220,000 retained faces.

## Architecture

Introduce `IncrementalTransitionCollisionState` in the transition module. It
owns the current candidate contributions, exposed representatives, persistent
self-collision index, cached static-obstacle hits, and collision contact graph.

The state has two operations:

```cpp
static Result<IncrementalTransitionCollisionState, TransitionBoundaryError>
build(const TransitionBoundaryInput &input);

Result<TransitionCollisionUpdate, TransitionBoundaryError> update(
    const TransitionBoundaryInput &next,
    const std::vector<LayerBoundaryOwnerKey> &changed_owners);
```

`build()` is the first-iteration full construction. `update()` creates a
working copy, applies all removals and insertions, validates diagonal
requirements, updates collisions, and publishes only after every step
succeeds.

`TransitionCollisionUpdate` contains:

```cpp
struct TransitionCollisionUpdate
{
    TransitionCollisionReport report;
    std::vector<OwnedBoundaryTriangle> exposed_boundary;
    IncrementalTransitionCollisionDiagnostics diagnostics;
};
```

The resolver uses `build()` for its first provisional transition. After a
rollback rebuilds provisional topology, it supplies the owner groups that may
have changed and calls `update()` instead of `checker.inspect()`.

## Stable Identities

### Owner key

Candidate contributions are grouped by:

```cpp
struct LayerBoundaryOwnerKey
{
    SurfaceFaceId source_face_id{};
    std::uint32_t layer{};
    BoundaryOwnerRole role{};
};
```

The rollback-high-face vector is payload, not identity. A changed owner group
replaces all triangles carrying the same key.

### Triangle key

Expose the existing orientation-independent `TriangleKey` based on three
sorted `CollisionVertexKey` tuples. Geometry is not part of this topological
key, matching current boundary cancellation behavior.

### Exposed primitive ID

Each currently exposed representative receives a state-local monotonic
`ExposedPrimitiveId`. IDs are not reused during one resolver call. The ID maps
an index primitive back to its complete `OwnedBoundaryTriangle` and owner key.
Each exposed triangle uses its own collision group so queries can ignore only
the query primitive, not every triangle owned by the same patch.

## Candidate Contribution State

The state maintains:

```text
owner key -> vector<ContributionId>
contribution ID -> OwnedBoundaryTriangle + TriangleKey
TriangleKey -> ordered vector<ContributionId>
TriangleKey -> optional exposed primitive ID
```

The ordered contribution vector makes the full assembler's representative rule
explicit: after stable sorting by `TriangleKey`, an odd-sized bucket exposes
the earliest contribution in provisional candidate order. Stage 1 changes the
full assembler from `std::sort` to `std::stable_sort`, adds representative-owner
tests, and establishes this deterministic contract before the incremental
state is enabled. Existing integration and Benchmark parity must pass after
that change. When a changed owner is replaced, contributions receive their
order from the new provisional input, so both paths choose the same owner and
metadata.

For each affected `TriangleKey`, an update compares the old and new exposed
representative:

- even to even: no exposed primitive;
- even to odd: insert the new representative;
- odd to even: remove the old representative;
- odd to odd with the same complete triangle and owner: preserve it;
- odd to odd with changed geometry or owner: replace it.

This explicitly handles an internal triangle becoming exposed after another
owner is removed.

## Determining Changed Owners

Correctness does not rely solely on rollback source IDs. After each new full
provisional build, the resolver creates owner-group fingerprints from:

- owner key and rollback-high-face payload;
- triangle vertex keys and coordinates;
- sliding-region metadata;
- physical-edge mask and complete-face exemptions;
- sliding-column data;
- diagonal requirements associated with the source face;
- resolved topology entries relevant to the owner.

It compares these fingerprints with the previous provisional state. Added,
removed, or unequal groups form `changed_owners`. This ensures neighboring
side transitions and top caps are updated even when their own source face was
not directly rolled back.

For the first implementation, fingerprints may be exact structural
comparisons rather than hashes. This keeps collision correctness independent
of hash collision behavior. The total provisional is already constructed, so
linear grouping is acceptable and much cheaper than spatial rescanning.

## Persistent Self-Collision State

The state owns an `IncrementalCollisionIndex`. Every exposed triangle is stored
as a separate collision group keyed by its `ExposedPrimitiveId`.

An update proceeds transactionally:

1. copy the current index and state metadata;
2. erase every removed or replaced exposed primitive group;
3. insert every added or replaced exposed primitive group;
4. remove contact-graph edges incident to removed/replaced primitives;
5. query every added/replaced primitive against the complete working index;
6. add every illegal contact as a canonical ordered primitive-ID pair;
7. publish the working state only after all insertions and queries succeed.

Unchanged-to-unchanged contacts remain valid because neither triangle geometry
nor collision policy changed. Changed-to-unchanged and changed-to-changed
contacts are recomputed after all new primitives have been inserted, so
simultaneous changes see each other's new geometry.

## Static Obstacle State

For each exposed primitive, cache whether it illegally contacts:

- the original surface;
- the historical exposed-boundary index;
- the prior-transition boundary;
- the sliding-surface constraints.

The existing special permissions remain unchanged, including own zero-layer
caps, own regular sources, historical-top ownership, equal prior-transition
triangle keys, and sliding-region exemptions.

Static obstacle queries run only for added or replaced exposed primitives.
Results for unchanged primitives remain valid because all external indices and
the triangle geometry are immutable during one resolver call.

The first implementation extracts the existing per-triangle obstacle logic
from `findRollbackFacesImpl()` into one shared helper used by both the full
checker and incremental state. No collision rule is duplicated.

## Contact Graph and Report Materialization

The state stores:

```text
set<pair<ExposedPrimitiveId, ExposedPrimitiveId>> self contacts
ExposedPrimitiveId -> static obstacle hit flag
ExposedPrimitiveId -> OwnedBoundaryTriangle
```

To produce a report:

1. visit every primitive with a static obstacle hit;
2. visit both endpoints of every self-contact edge;
3. append each unique `LayerBoundaryOwner` using the existing owner identity;
4. collect its rollback-high-face IDs;
5. sort and deduplicate rollback IDs exactly as the full checker does.

Report materialization is linear in the number of active collision records,
not the number of all possible triangle pairs.

## Diagonal Requirements

Conflicting `LayerDiagonalRequirement` entries remain an error. The state keeps
the latest complete diagonal-requirement vector and validates it before any
incremental mutation is published. Diagonal validation is initially full
because the vector is small relative to boundary collision work. It can be
indexed separately later if measurements justify it.

## Resolver Integration

The resolver owns one optional incremental state for its entire outer rollback
loop:

1. First provisional: call `IncrementalTransitionCollisionState::build()` and
   use its report.
2. External distance probes: continue using the existing specialized patch
   index; they do not publish into the rollback-iteration state.
3. Final external geometry: update or rebuild the iteration state to the final
   provisional before choosing rollback IDs.
4. If rollback is non-empty, build the next provisional and call state
   `update()` with exact changed owner groups.
5. If rollback is empty, return the state's exposed-boundary snapshot directly
   rather than calling `assembleExposedBoundary()` again.

Stage 2 keeps external-distance final verification on the full checker and
enables incremental rollback reuse only for iterations where external geometry
did not change. Stage 3 extends the same transactional state update to final
external-patch geometry and removes that Release-only full verification.

## Parity and Fallback

During development and Debug builds, every incremental update also runs the
existing full checker and full exposed-boundary assembler. It compares:

- sorted rollback IDs;
- colliding owner keys;
- exposed `TriangleKey` multiplicity and representative owner;
- exposed triangle coordinates and metadata.

Any mismatch returns a dedicated transition error in tests rather than silently
using the incremental result. Release builds do not run the full parity scan.

The production API retains an explicit full-rebuild operation for:

- initial construction;
- changed collision-policy inputs;
- detected state/version mismatch;
- callers that do not provide changed-owner information.

Fallback is correctness recovery, not the normal rollback path, and is counted
in diagnostics.

## Diagnostics

Record separately:

- initial full-state build time;
- owner-group diff time;
- changed owner count;
- affected `TriangleKey` count;
- exposed inserts, erases, and replacements;
- static obstacle queries;
- self-collision queries and exact tests;
- contact-graph update time;
- report materialization time;
- full parity time in Debug;
- fallback rebuild count.

This replaces the ambiguous `initial_scan_ms` with evidence about actual
incremental work.

## Testing

Unit fixtures must cover:

1. removing one of two equal-key contributions re-exposes the survivor;
2. inserting the second equal-key contribution hides an exposed triangle;
3. odd-to-odd representative replacement updates owner and geometry;
4. unchanged self-contact edges survive unrelated updates;
5. edges incident to removed primitives disappear;
6. simultaneously changed groups collide using all-new geometry;
7. a changed primitive detects a previously unseen unchanged obstacle;
8. static obstacle permissions match the full checker;
9. sliding and prior-transition contacts match the full checker;
10. insertion failure does not publish partial state;
11. incremental and full reports/exposed boundaries agree across multiple
    rollback iterations;
12. diagonal conflicts return the existing error.

Integration verification runs all transition, regular-layer, and collision
tests. Release Benchmark must retain 215,050 layer-20 cells and materially
reduce the two approximately 7-second rollback-iteration scans.

## Delivery Stages

### Stage 1: Incremental exposed-boundary contributions

Implement owner grouping, `TriangleKey` buckets, transactional replacement,
and Debug parity with `assembleExposedBoundary()`. Collision reporting still
uses the full checker. This validates deletion/re-exposure semantics.

### Stage 2: Persistent collision index and contact graph

Add per-primitive groups, cached static hits, self-contact edges, incremental
report materialization, and full-checker Debug parity. Enable incremental scans
for rollback iterations without external geometry changes.

### Stage 3: External-final integration and Release benchmark

Apply final external patch geometry transactionally to the same state, remove
the redundant final full scan in Release, run parity tests, and measure the
Benchmark.

## Out of Scope

- Avoiding the full provisional transition build itself.
- Incremental corner-suppression propagation.
- Changing external distance-search bisection policy.
- Changing collision tolerances or legal-contact classification.
- Parallel collision queries.
