# Incremental Transition Collision State Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reuse exposed-boundary and collision state across transition resolver rollback iterations so only changed owner groups are reassembled and rechecked.

**Architecture:** A new `IncrementalTransitionCollisionState` groups provisional triangles by stable owner key, maintains `TriangleKey` contribution buckets and exposed representatives, then incrementally updates a persistent collision index, static-obstacle cache, and self-contact graph. Debug builds compare every incremental result with the existing full checker before the resolver consumes it.

**Tech Stack:** C++17, CMake/CTest, `IncrementalCollisionIndex`, existing transition boundary policies, and `Result` error propagation.

## Global Constraints

- Preserve exposed triangle metadata, colliding owners, sorted rollback IDs, resolved topology, and final cell counts.
- Simultaneously changed groups must all be inserted before collision queries.
- Removed contributions can re-expose an unchanged owner's triangle and must trigger a new query.
- State updates are transactional; failures publish no partial mutation.
- Debug parity mismatch is a test-visible error; Release does not pay for full parity scans.
- Benchmark acceptance remains 20 layers, first height `0.1`, growth ratio `1.2`, maximum skewness `1.0`, and 215,050 cells on layer 20.

---

### Task 1: Deterministic triangle and owner identities

**Files:**
- Modify: `include/boundary_mesh/transition/transition_boundary_checker.hpp`
- Modify: `src/transition/transition_boundary_checker.cpp`
- Modify: `tests/unit/transition/transition_boundary_checker_test.cpp`

**Interfaces:**
- Produces: `LayerBoundaryOwnerKey`, equality/order helpers, and `TransitionTriangleKey`.
- Produces: `transitionTriangleKey(const OwnedBoundaryTriangle &)`.
- Establishes: stable candidate-order representative selection for equal keys.

- [ ] Add a failing test containing three equal-key triangles with different owners and assert that `assembleExposedBoundary()` returns the earliest candidate contribution.
- [ ] Build and run `boundary_mesh_transition_boundary_checker_test`; verify failure demonstrates the current unstable representative contract or missing public key API.
- [ ] Move the private triangle-key type/function into the public transition boundary API, add `LayerBoundaryOwnerKey`, and change boundary assembly to `std::stable_sort` by key.
- [ ] Verify checker, resolver, and transition pipeline tests.
- [ ] Commit as `refactor: stabilize transition boundary identities`.

### Task 2: Incremental exposed-boundary contribution state

**Files:**
- Create: `include/boundary_mesh/transition/incremental_transition_collision_state.hpp`
- Create: `src/transition/incremental_transition_collision_state.cpp`
- Create: `tests/unit/transition/incremental_transition_collision_state_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `IncrementalTransitionCollisionState::buildBoundary(const TransitionBoundaryInput &)`.
- Produces: `replaceBoundary(const TransitionBoundaryInput &, const std::vector<LayerBoundaryOwnerKey> &)`.
- Produces: `exposedBoundary()` and boundary diagnostics.

- [ ] Add failing tests for even-to-odd re-exposure, odd-to-even hiding, odd-to-odd representative replacement, unrelated-owner preservation, and diagonal-conflict rollback.
- [ ] Register the new test target and verify RED because the state API is absent.
- [ ] Implement owner contribution maps, stable candidate ordinals, `TransitionTriangleKey` buckets, and deterministic exposed representatives.
- [ ] Implement updates on a working copy; validate diagonal requirements before publishing.
- [ ] Compare the state output field-by-field with `assembleExposedBoundary()` after every fixture update.
- [ ] Run the new test plus existing checker/resolver tests.
- [ ] Commit as `feat: update transition exposed boundary incrementally`.

### Task 3: Shared static-obstacle query and persistent contact graph

**Files:**
- Modify: `include/boundary_mesh/transition/transition_boundary_checker.hpp`
- Modify: `src/transition/transition_boundary_checker.cpp`
- Modify: `include/boundary_mesh/transition/incremental_transition_collision_state.hpp`
- Modify: `src/transition/incremental_transition_collision_state.cpp`
- Modify: `tests/unit/transition/incremental_transition_collision_state_test.cpp`

**Interfaces:**
- Produces: shared single-triangle static-obstacle inspection used by full and incremental paths.
- Produces: `build(const TransitionBoundaryInput &)` and `update(...)` returning `TransitionCollisionUpdate`.
- Produces: persistent `IncrementalCollisionIndex`, static-hit cache, and canonical self-contact edge set.

- [ ] Add failing tests for cached unchanged contacts, incident-edge removal, simultaneous new geometry, unseen unchanged obstacles, original/historical/prior/sliding permissions, and failed-update atomicity.
- [ ] Verify RED against the boundary-only state.
- [ ] Extract the full checker's per-triangle static obstacle rules without changing their order or exemptions.
- [ ] Store one collision group per exposed primitive; on update erase all removed/replaced groups, insert all new groups, then query every new primitive.
- [ ] Maintain canonical contact pairs and static-hit flags, then materialize unique owners and sorted rollback IDs.
- [ ] Add Debug parity helpers comparing report and exposed boundary with the full checker.
- [ ] Run transition, sliding, and incremental collision-index tests.
- [ ] Commit as `feat: maintain transition collision contacts incrementally`.

### Task 4: Resolver rollback integration

**Files:**
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `tests/unit/transition/layer_transition_resolver_test.cpp`
- Modify: `tests/integration/incremental_layer_transition_pipeline_test.cpp`

**Interfaces:**
- Consumes: incremental collision state from Task 3.
- Produces: first-iteration full state build and later rollback-iteration incremental updates when external geometry is unchanged.

- [ ] Extend a resolver fixture to require at least three rollback iterations and record full-state builds versus incremental updates; assert one full build and at least two updates with unchanged final output.
- [ ] Verify RED because the resolver still invokes `checker.inspect()` every iteration.
- [ ] Keep the collision state outside the resolver outer loop; build it for iteration one and structurally diff owner groups after subsequent provisional builds.
- [ ] Use incremental reports only when external distance geometry was unchanged; retain the full path for external-changed iterations at this stage.
- [ ] Return the state's exposed boundary on stable completion instead of assembling it again.
- [ ] Add timings for owner diff, changed owners, affected keys, index operations, and report materialization.
- [ ] Run all transition/growth/collision tests.
- [ ] Commit as `perf: reuse collision state across transition rollbacks`.

### Task 5: Final external-patch geometry integration

**Files:**
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `tests/unit/transition/layer_transition_resolver_test.cpp`

**Interfaces:**
- Consumes: final locally rebuilt external patch records.
- Produces: one transactional update from initial provisional geometry to final external geometry, eliminating the Release final full scan.

- [ ] Add failing tests where several external patches change together, one becomes safe, and another still collides; compare incremental and full reports.
- [ ] Verify RED while the resolver still calls the final full checker.
- [ ] Update the shared collision state with final external owner groups after bisection and use its report for rollback decisions.
- [ ] Preserve full checker/assembler parity in Debug only.
- [ ] Run resolver, checker, transition pipeline, and collision-index tests.
- [ ] Commit as `perf: verify final external patches incrementally`.

### Task 6: Release verification and Benchmark

**Files:**
- No tracked production changes; store logs under the existing Benchmark case directory.

**Interfaces:**
- Produces: fresh parity and performance evidence.

- [ ] Run `git diff --check`, focused Debug tests, and a Release CLI build.
- [ ] Temporarily use the existing orientation helper for Benchmark only; run 20 layers with `0.1`, `1.2`, and skewness `1.0`.
- [ ] Require layer 20 to add exactly 215,050 cells.
- [ ] Compare rollback scan work with the current layer-20 two-scan total of 13.823 seconds and layer-19 scan total of 16.166 seconds.
- [ ] Remove the temporary orientation call, rebuild the official Release CLI, rerun focused tests, and confirm only pre-existing user changes remain.
