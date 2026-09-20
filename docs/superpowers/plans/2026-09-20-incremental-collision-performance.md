# Incremental Collision Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the quadratic exposed-boundary update and reduce repeated collision geometry work while preserving existing collision decisions.

**Architecture:** Introduce a reusable per-layer outer-shell batch, use a direct boundary-key registry with transactional churn-aware rebuilding, and run self-collision through owner-level broad phase before unchanged exact triangle contact tests. Obstacle and self-collision stages consume the same prepared geometry; empty sliding geometry bypasses sliding work.

**Tech Stack:** C++20, CMake/CTest, existing `Result`, `CollisionTriangle`, `IncrementalCollisionIndex`, and BVH utilities.

## Global Constraints

- Keep the default growth ratio at `1.2`.
- Preserve `hasIllegalTriangleContact()` semantics and deterministic stop decisions.
- Do not permanently add automatic orientation unification to the CLI as part of this change.
- Do not alter the stepper until collision and boundary-update timings have been remeasured.
- Preserve unrelated uncommitted CLI, generator, integration-test, submodule, VS Code, and configuration changes.

---

### Task 1: Direct boundary registry and transactional bulk rebuild

**Files:**
- Modify: `include/boundary_mesh/growth/exposed_boundary.hpp`
- Modify: `src/growth/exposed_boundary.cpp`
- Modify: `tests/unit/growth/exposed_boundary_test.cpp`

**Interfaces:**
- Consumes: `BoundaryFaceKey`, `ExposedBoundaryUpdate`, and `IncrementalCollisionIndex`.
- Produces: `BoundaryFaceKeyLess`, `ExposedBoundaryApplyDiagnostics`, and `ExposedBoundaryTracker::lastApplyDiagnostics()`.

- [ ] **Step 1: Write failing registry and mode-boundary tests**

  Extend `exposed_boundary_test.cpp` with tests that seed five faces, replace zero/one faces through incremental mode, replace exactly one of five faces through bulk mode (the exact 20% boundary), and verify duplicate insert or missing erase leaves the face keys and collision query results unchanged. Assert diagnostics expose erased/inserted counts and the selected mode.

- [ ] **Step 2: Verify the new tests fail for missing diagnostics and rollback behavior**

  Run `cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test && ctest --test-dir build -C Debug -R boundary_mesh_exposed_boundary_test --output-on-failure` and confirm failure is caused by the missing API/behavior.

- [ ] **Step 3: Add normalized-key ordering and replace the linear vector**

  Define a lexicographic `BoundaryFaceKeyLess` over `vertex_count` and all normalized vertex keys. Store collision groups as `std::map<BoundaryFaceKey, CollisionGroupId, BoundaryFaceKeyLess>` so erase and membership are logarithmic and deterministic.

- [ ] **Step 4: Implement prepare-before-publish apply modes**

  Validate every erase and insert against a prospective final face map before mutation. If `(erase_count + insert_count) * 5 >= max(current_face_count, 1)`, build the complete replacement index and registry in local variables and publish by move only after success; otherwise prepare triangle groups and perform direct registry erases/inserts. Record preparation time, index-update time, counts, and mode.

- [ ] **Step 5: Run focused and dependent tests**

  Run the exposed-boundary, farfield-boundary, transition-boundary, and regular-layer growth tests with `ctest --test-dir build -C Debug -R "(exposed_boundary|farfield_boundary|transition_boundary|regular_layer_growth)" --output-on-failure`.

- [ ] **Step 6: Commit the isolated change**

  Stage only the three task files and commit with `perf: rebuild high-churn exposed boundary`.

### Task 2: Reusable outer-shell layer batch

**Files:**
- Create: `include/boundary_mesh/growth/layer_boundary_batch.hpp`
- Create: `src/growth/layer_boundary_batch.cpp`
- Create: `tests/unit/growth/layer_boundary_batch_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `std::vector<LayerBoundaryCandidate>` and stable owner indices.
- Produces: `Result<LayerBoundaryBatch, SpatialError> LayerBoundaryBatch::build(...)`, owner records containing retained shell triangles/AABBs, and diagnostics for owner, triangle, and omitted-side counts.

- [ ] **Step 1: Write failing shell-selection tests**

  Test an isolated quad retains top plus four sides, two active adjacent quads omit both copies of their shared side, an active face beside a missing/stopped owner retains the interface side, triangle/quad adjacency uses the same canonical edge key, and three accepted faces sharing an edge return a non-manifold error.

- [ ] **Step 2: Verify the test target fails because the batch API is absent**

  Configure/build the new target and confirm the compiler reports the missing header/type.

- [ ] **Step 3: Implement canonical edge incidence and immutable batch records**

  Build an undirected bottom-edge table keyed by ordered `CollisionVertexKey` pairs. Emit every top split; emit a side split only for incidence one; omit both copies for incidence two; reject incidence above two. Precompute each triangle AABB and the union AABB for each owner.

- [ ] **Step 4: Verify focused tests and existing candidate geometry tests**

  Run `ctest --test-dir build -C Debug -R "(layer_boundary_batch|layer_collision_checker|exposed_boundary)" --output-on-failure`.

- [ ] **Step 5: Commit the isolated change**

  Commit the new batch implementation and build registration with `feat: build reusable layer boundary shell`.

### Task 3: Owner-level self-collision broad phase

**Files:**
- Modify: `include/boundary_mesh/spatial/batch_self_collision_detector.hpp`
- Modify: `src/spatial/batch_self_collision_detector.cpp`
- Modify: `tests/unit/spatial/batch_self_collision_detector_test.cpp`

**Interfaces:**
- Consumes: owner records from `LayerBoundaryBatch` or an equivalent span of `{owner_id, owner_aabb, triangles}`.
- Produces: `BatchSelfCollisionDetector::detectOwners(...)` with deterministic illegal owner IDs and diagnostics for owner pairs, AABB rejects, and exact triangle tests.

- [ ] **Step 1: Write failing owner-pair tests**

  Test separated owners produce no triangle exact tests, overlapping owners are processed once regardless of input order, multiple colliding triangle pairs report each owner once, and invalid owner AABBs return an error.

- [ ] **Step 2: Verify RED with the focused detector target**

  Run the detector test target and confirm failure is due to the missing owner-level API.

- [ ] **Step 3: Implement deterministic two-level broad phase**

  Generate unordered candidate owner pairs from owner AABBs, sort/unique them, then expand only those pairs into existing triangle-AABB and `hasIllegalTriangleContact()` checks. Do not add adjacency exemptions or change exact-contact classification.

- [ ] **Step 4: Run detector and collision-index tests**

  Run `ctest --test-dir build -C Debug -R "(batch_self_collision_detector|collision_index|triangle_contact)" --output-on-failure`.

- [ ] **Step 5: Commit the isolated change**

  Commit with `perf: broad phase self collision by owner`.

### Task 4: Reuse the batch through obstacle, self-collision, and generator stages

**Files:**
- Modify: `include/boundary_mesh/growth/layer_collision_checker.hpp`
- Modify: `src/growth/layer_collision_checker.cpp`
- Modify: `include/boundary_mesh/spatial/sliding_intersection_index.hpp`
- Modify: `src/spatial/sliding_intersection_index.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/unit/growth/layer_collision_checker_test.cpp`
- Modify: `tests/unit/spatial/sliding_intersection_index_test.cpp`

**Interfaces:**
- Consumes: one `LayerBoundaryBatch` created after coordination and an optional rebuilt batch only after owner compaction changes the accepted set.
- Produces: checker overloads accepting the batch, `SlidingIntersectionIndex::empty()`, and detailed per-layer diagnostics.

- [ ] **Step 1: Write failing reuse and empty-sliding tests**

  Add tests showing prebuilt batch overloads match legacy checker results and an empty sliding index performs zero sliding queries while preserving obstacle decisions.

- [ ] **Step 2: Verify focused tests fail for the missing overloads/short circuit**

  Build and run the layer-collision and sliding-index targets and confirm the expected API failures.

- [ ] **Step 3: Make checker stages consume precomputed triangles and AABBs**

  Add batch-taking overloads; keep legacy overloads as compatibility wrappers that build a batch once. Route self-collision to `detectOwners()`. Return the same `LayerStepResult` ordering and stop reasons.

- [ ] **Step 4: Add the empty sliding fast path**

  Implement `empty()` as `primitiveCount() == 0` and bypass sliding reconstruction/query logic before entering per-owner work when either the surface set or index is empty.

- [ ] **Step 5: Wire one-batch-per-layer construction and diagnostics**

  In `regular_layer_generator.cpp`, construct the shared batch once after candidate coordination, reuse it for obstacle/self collision, and rebuild only if filtering changes the accepted owner set. Preserve existing stage labels and add batch-build, broad/exact, registry/rebuild, compaction timings plus relevant counts.

- [ ] **Step 6: Run focused growth and transition suites**

  Run `ctest --test-dir build -C Debug -R "(layer_collision_checker|sliding_intersection_index|regular_layer|transition)" --output-on-failure`.

- [ ] **Step 7: Commit the integrated change**

  Commit only task files with `perf: reuse layer collision boundary batch`.

### Task 5: Release verification and performance evidence

**Files:**
- Modify only if a regression is found: files owned by Tasks 1-4 and their tests.
- Record logs outside tracked source under each case output directory.

**Interfaces:**
- Consumes: Release CLI and the existing case configurations.
- Produces: parity evidence and per-stage timings for 2dot5, anisotropic, and Benchmark.

- [ ] **Step 1: Run formatting/diff checks and the complete Debug suite**

  Run `git diff --check`, build all Debug targets, and run `ctest --test-dir build -C Debug --output-on-failure`. Fix regressions with a new failing test before production changes.

- [ ] **Step 2: Build Release**

  Run the repository's configured Release build command and confirm the CLI target exits successfully.

- [ ] **Step 3: Run 2dot5 parity case**

  Use the existing 2dot5 reference parameters. Compare layer cell counts, stopped faces/reasons, exposed-boundary count, and output validity against the last accepted result.

- [ ] **Step 4: Run anisotropic parity case**

  Run 20 layers with first height `0.1` and growth ratio `1.2`; compare the same topology/growth outputs and retain the timing log.

- [ ] **Step 5: Run Benchmark to completion**

  Run `Benchmark.cgns` for 20 layers, first height `0.1`, growth ratio `1.2`, maximum skewness `1.0`. Use only the separately scoped/test-only orientation preparation needed by this known inconsistent input. Verify it passes layer 2 boundary update and completes all requested layers.

- [ ] **Step 6: Summarize fresh bottlenecks**

  Report total and per-layer time for batch construction, obstacle broad/exact work, self owner broad/exact work, boundary registry/rebuild, compaction, and stepper; identify the largest remaining stage from measured evidence.

- [ ] **Step 7: Final verification commit**

  If verification required fixes, commit them separately after the full suite and all three Release cases pass. Otherwise leave performance logs untracked and report the exact commands/results.
