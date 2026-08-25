# BLMesh Multi-Normal Intersection Parity Design

## Goal

Replace the current generic collision-based multi-normal length resolver with a function-by-function port of BLMesh's MNormal intersection and length-resolution path. For the same triangulated PLS surface and initial length field, both implementations must produce the same shared-point classification, bad-point set, retry sequence, and accepted branch lengths.

## Scope

The port covers `InitLengthField`, `RebuildPointNeighbors`, `SmoothLengthField`, `BuildLayerPoints`, `CheckSurfaceIntersection`, `CheckOuterSurfaceIntersection`, `IntersecChecker::checkIntersect`, `ShrinkLengthField`, `ZeroLengthField`, and `ResolveLengthField`.

The existing split planner, topology builder, affected-Quad triangulation, tetrahedral transition builder, debug writer, and regular-layer merger remain unchanged except where their interfaces must consume the ported resolver result.

## Porting Rule

The BLMesh control flow, loop order, constants, and geometric-contact semantics are copied structurally. Only names, ownership, error transport, and project point/index types may change. The implementation must not substitute `CollisionIndex::queryIllegalContacts()` for BLMesh's `IntersecChecker::checkIntersect()` semantics.

## Geometric Contact Semantics

Triangle vertices are compared by geometric coordinates, matching BLMesh. For each query/candidate triangle pair:

- three shared coordinates mean the same geometric triangle and are skipped;
- two shared coordinates mean a shared edge and are legal without further testing;
- one shared coordinate triggers the two BLMesh line/triangle tests using the non-shared edges;
- zero shared coordinates triggers BLMesh's full 3D triangle-overlap test.

The first parity implementation retains BLMesh's exact coordinate equality. A tolerance-based variant is outside this change unless a separate parity fixture demonstrates the need.

The spatial tree may be replaced by an existing acceleration structure only if tests prove identical candidate-pair and result semantics. The default implementation ports the BLMesh query behavior directly to avoid another semantic mismatch.

## Length Resolution

The initial vector contains the requested first-layer length only for duplicated lower IDs created by multi-normal splitting. Non-split points have zero length. Neighbor rebuilding and the `1.1` upper-bound smoother follow BLMesh.

Each resolver iteration builds the displaced outer surface and collects `bad_points` from intersecting faces. For iterations 0 through 19, only those bad point lengths are multiplied by `0.8`, followed by length smoothing.

After the shrink iterations, BLMesh's zero-length step is performed. In ALM/pre-transition mode, reaching this fallback means the multi-normal attempt fails and the caller returns the unchanged single-normal front; it must not publish coincident split branches as a successful transformed front. Non-ALM behavior retains BLMesh's one zero-length retry.

## Errors and Result

The resolver returns accepted lengths, shrink count, and zero-retry state on success. Program/data inconsistencies remain `MultiNormalError`. A geometric ALM fallback is represented separately from a program error so the generator can return an unapplied, unchanged front rather than abort the entire boundary-layer workflow.

## Verification

Tests are written before production changes and must fail against the current resolver.

Required fixtures:

1. Identical geometric triangles with different layer-local IDs are legal.
2. Triangles sharing a complete geometric edge are legal.
3. Triangles sharing one point use the BLMesh non-shared-edge tests.
4. Non-adjacent triangles with no shared points detect a proper intersection.
5. Bad-point shrinking changes only the BLMesh bad-point set.
6. The PLS point `(205.9503, -600, 3.3557019)` produces three nonzero multi-normal branches at initial length `0.01`, matching the original BLMesh reference.
7. An unresolved ALM candidate falls back instead of accepting zero-height branches.

The real-case acceptance run uses `2dot5_cf.pls`, multi-normal enabled, initial height `0.01`, and regular-layer count zero. The original BLMesh reference reports `Loop 0, bad point size = 0`; the port must do the same for the target local patch and must preserve the three nonzero branches.

## Delivery Boundary

All production and test changes stay on `codex/multi-normal-topology-transition`. Temporary real-case harnesses are removed before completion. Reference VTK outputs may remain under ignored build or external case-output directories, but no machine-specific absolute path is committed.
