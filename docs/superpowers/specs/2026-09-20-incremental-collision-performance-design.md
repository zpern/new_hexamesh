# Incremental Collision Performance Design

## Goal

Remove the second-layer quadratic boundary-update stall and substantially reduce per-layer collision cost while preserving the existing `CollisionTriangle` exact-contact semantics, accepted cell counts, and stop decisions.

The primary acceptance case is `Benchmark.cgns` with 20 layers, first height `0.1`, growth ratio `1.2`, maximum skewness `1.0`, and a Release build. The complete run must finish. The 2dot5 and anisotropic cases remain regression references.

## Evidence and Reference Model

The current Benchmark layer contains approximately 225,498 candidate faces. Layer 2 completes the timed collision stages in about 54 seconds and then stalls inside `ExposedBoundaryTracker::apply()`. For every erased boundary face, that function linearly searches and erases from `collision_groups_`, producing quadratic behavior.

The blmesh reference keeps stable triangle IDs in a persistent Octree, directly removes old triangles, inserts new top triangles and only genuinely exposed side triangles, scans changed leaves for large updates, and rebuilds only when the tree is overloaded. This design adopts those data-flow properties without replacing the current exact triangle-contact classifier.

## Architecture

### One boundary batch per layer

Create the layer boundary candidates once after candidate coordination and reuse that batch for obstacle collision, self-collision, and committed exposed-boundary updates. The batch owns face geometry, stable owner IDs, precomputed face keys, triangle splits, and AABBs. No stage rebuilds equivalent `BoundaryFace` or `CollisionTriangle` vectors independently.

### Direct boundary registry

Replace the linear `collision_groups_` vector lookup with an ordered or hashed registry from `BoundaryFaceKey` to `CollisionGroupId`. Key ordering or hashing must use all normalized topology keys and `vertex_count`.

Small updates erase and insert groups through direct registry lookup. The operation validates every requested erase and every prospective insert before mutating tracker state.

### Churn-aware transactional rebuild

When changed faces are at least 20 percent of the current registry, construct the complete post-update face map and a replacement `IncrementalCollisionIndex` off to the side. Publish the new face set, registry, next group ID, and index only after the build succeeds. This turns Benchmark's near-total layer replacement into one bulk build instead of hundreds of thousands of individual operations.

Updates below 20 percent retain the incremental path. That path must also be transactional: validate all keys and collision triangles first, and do not leave a partially applied tracker after a failure.

### Outer-shell candidate geometry

Build an undirected edge-incidence table for active candidate faces. Each top face remains collision geometry. A candidate side is included only when its bottom edge has no simultaneously accepted neighboring candidate across that edge. Thus sides on the exterior of the active patch and interfaces against stopped faces remain, while the coincident side shared by two accepted neighboring cells is omitted.

This is the same topological rule used by blmesh's `used_by_neigh_front`. It does not suppress geometry based on coordinate proximity. Non-manifold candidate edges return an error. Mixed triangle and quad fronts use the same canonical edge keys.

### Two-level self-collision broad phase

Build one AABB per candidate owner from its retained outer-shell triangles. Query unordered owner pairs once. Only overlapping owner pairs expand into triangle-pair tests using the existing `hasIllegalTriangleContact()` function. Topologically adjacent owners remain subject to the same legal-contact rules already encoded in `CollisionTriangle`; the broad phase does not grant new exemptions.

The first implementation remains deterministic and serial. Parallel owner-pair processing is a later step after result parity is established.

### Batched obstacle collision

Obstacle checking consumes the shared boundary batch and its precomputed triangle AABBs. It queries the original-surface and exposed-boundary indexes without reconstructing candidate geometry.

If `SlidingSurfaceSet` is empty, the complete sliding-intersection path is skipped. Once serial parity tests pass, independent owner queries may run in parallel. Mutable diagnostics must be accumulated per worker and reduced afterward rather than written through shared counters.

### Stepper scope

The stepper is not changed in the first implementation wave. After collision and boundary-update work is complete, fresh Benchmark measurements determine whether its approximately 10-second cost warrants separate parallelization. This prevents unrelated growth behavior changes from being mixed with collision fixes.

## Diagnostics

Report at least these timings per layer:

- boundary batch construction;
- obstacle broad phase and exact tests;
- self-collision owner broad phase and triangle exact tests;
- exposed-boundary registry preparation;
- incremental index update or bulk index rebuild;
- front compaction.

Report counts for candidate owners, retained shell triangles, omitted shared sides, owner pairs, triangle exact tests, erased groups, inserted groups, and whether the update selected incremental or bulk mode.

## Error Handling

All public operations continue to use `Result`. Invalid topology references, duplicate or missing boundary keys, non-manifold active edges, invalid AABBs, and failed replacement-index builds return errors. Tracker state and committed volume/front state remain unchanged when preparation or application fails.

## Verification

Unit tests cover direct registry lookup, small incremental updates, the exact 20-percent mode boundary, large transactional rebuilds, failure rollback, external-side selection, mixed triangle/quad adjacency, non-manifold active edges, deterministic owner-pair generation, and empty-sliding short-circuiting.

Integration tests compare accepted/stopped faces, stop reasons, cells, and exposed-boundary output before and after optimization on existing fixtures. Existing collision and transition suites must pass.

Performance verification uses Release builds:

1. 2dot5: existing reference parameters and output parity.
2. anisotropic: 20 layers, first height `0.1`, growth ratio `1.2`, output parity.
3. Benchmark: 20 layers, first height `0.1`, growth ratio `1.2`, maximum skewness `1.0`; the complete run must finish and must not stall after layer 2 boundary update.

The temporary CLI orientation call used during diagnosis is not part of this performance change. Benchmark execution may explicitly invoke the orientation helper before topology construction only when separately approved or through a test-only driver.

