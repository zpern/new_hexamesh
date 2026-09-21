# Incremental External Patch Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace full provisional-transition rebuilds inside external distance search with local external-patch rebuilds and transactional collision-group replacement.

**Architecture:** The full builder remains the source of truth but gains a local external-only entry point that executes the same patch construction branches for selected source-face IDs. The resolver caches the first full provisional result, replaces selected patch records locally during distance probes, and queries the new triangles against a persistent collision environment.

**Tech Stack:** C++17, CMake/CTest, existing transition templates, `Result`, `IncrementalCollisionIndex`, and exact triangle-contact classification.

## Global Constraints

- Preserve final retained faces, rollback IDs, resolved topology, and cell counts.
- Query every new patch against the complete unchanged environment; previous collision partners are only an early-check optimization.
- Simultaneously changed patches must be checked using all-new geometry.
- Do not change corner suppression, transition templates, stepper behavior, or surface-orientation policy.
- Benchmark acceptance is 20 layers, first height `0.1`, growth ratio `1.2`, maximum skewness `1.0`, and 215,050 cells on layer 20.

---

### Task 1: Local external-patch construction

**Files:**
- Modify: `include/boundary_mesh/transition/provisional_transition_builder.hpp`
- Modify: `src/transition/provisional_transition_builder.cpp`
- Modify: `tests/unit/transition/transition_boundary_checker_test.cpp`

**Interfaces:**
- Consumes: the same current/candidate fronts, retained IDs, face sets, terminal-hexa callback, controls, and terminal candidates as `buildProvisionalTransition()` plus selected patch IDs.
- Produces: `buildProvisionalExternalPatches(...) -> ProvisionalLayerTransitionResult` containing only selected external-patch triangles/topology/forced rollbacks.

- [ ] **Step 1: Add a failing parity test**

  Build a fixture containing both regular geometry and at least two external patches. For scales `0.25`, `0.125`, and `0.0625`, compare each selected patch's triangles, generated point, owner metadata, and terminal decision between the full result and local result. Assert the local result contains no unselected owner.

- [ ] **Step 2: Verify RED**

  Build `boundary_mesh_transition_boundary_checker_test` and confirm compilation fails because `buildProvisionalExternalPatches` is absent.

- [ ] **Step 3: Share one internal builder implementation**

  Add an internal optional selected-external-ID filter. In external-only mode, skip regular candidate emission, skip unselected terminal candidates and transition-low faces before template construction, and emit only entries whose final decision is `ExternalPatch` or the selected patch's fallback decision. Keep all existing geometry/template helpers unchanged.

- [ ] **Step 4: Verify parity and existing transition tests**

  Run `ctest --test-dir build -C Debug -R "(transition_boundary_checker|incremental_layer_transition|triangle_side_transition)" --output-on-failure`.

- [ ] **Step 5: Commit**

  Commit the three task files with `feat: rebuild selected external patches`.

### Task 2: Transactional cached patch replacement

**Files:**
- Modify: `include/boundary_mesh/transition/layer_transition_resolver.hpp`
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `tests/unit/transition/layer_transition_resolver_test.cpp`

**Interfaces:**
- Consumes: new `LayerTransitionInput::build_external_patches` callback with the same selected-ID and controls contract as Task 1.
- Produces: cached patch replacement inside `LayerTransitionResolver::resolve()` and `ExternalPatchSearchDiagnostics` counters for full builds, local builds, replaced groups, and exact probe queries.

- [ ] **Step 1: Add failing resolver behavior tests**

  Extend the existing external fixtures with separate full/local build counters. Assert a search uses one full build per outer resolver iteration and multiple local builds, simultaneous patches see each other's new geometry, a collision with a previously unseen obstacle is detected, and a local build failure returns without publishing partial patch records.

- [ ] **Step 2: Verify RED**

  Build/run `boundary_mesh_layer_transition_resolver_test`; confirm failure comes from the missing callback/diagnostics.

- [ ] **Step 3: Partition and merge patch records**

  Add helpers that identify a patch by `owner.role == ExternalPatch && owner.source_face_id == id`, remove selected patch triangles/topology from a working provisional, then insert local replacements. Preserve immutable diagonal requirements and configure all external pointers after merging.

- [ ] **Step 4: Replace collision groups transactionally**

  Build the search index once from the initial assembled boundary. For each probe round, build every active patch locally first, copy the index/patch cache, erase all old active groups, insert all new groups, then query each new triangle ignoring its own stable group. Publish only after all operations succeed.

- [ ] **Step 5: Eliminate inner full builds**

  Use local replacement in halving and all 12 bisection steps. Materialize the final provisional from the immutable base plus cached final patch records. Retain the existing full-build fallback only when `build_external_patches` is not supplied, preserving compatibility for synthetic callers.

- [ ] **Step 6: Run focused tests and commit**

  Run resolver, boundary-checker, transition pipeline, and collision-index tests. Commit with `perf: update external search patches incrementally`.

### Task 3: Wire the real local builder and diagnostics

**Files:**
- Modify: `src/boundary_layer/incremental_boundary_layer_generator.cpp`
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `tests/integration/incremental_layer_transition_pipeline_test.cpp`

**Interfaces:**
- Consumes: `buildProvisionalExternalPatches()` and resolver diagnostics.
- Produces: production callback wiring and per-iteration timings/counts.

- [ ] **Step 1: Add a failing integration assertion**

  Use a transition fixture that requires external distance search and assert the resolver reports one full build, more than one local build, and identical retained faces/topology to the reference outcome.

- [ ] **Step 2: Verify RED**

  Run the incremental transition pipeline target and confirm missing production callback/counters cause failure.

- [ ] **Step 3: Wire callback and stage diagnostics**

  Capture the same fronts, face sets, terminal-hexa callback, terminal candidates, and controls in `build_external_patches`. Print full-build count/time, local-build count/time, replacement time, query time, searched patch count, and probe rounds.

- [ ] **Step 4: Run all transition/growth tests and commit**

  Run `ctest --test-dir build -C Debug -R "(regular_layer|transition|collision)" --output-on-failure`, then commit with `perf: wire incremental external patch search`.

### Task 4: Release parity and Benchmark

**Files:**
- No tracked output files; store generated VTK artifacts under the existing case directories.

**Interfaces:**
- Consumes: Release CLI and accepted case parameters.
- Produces: parity and timing evidence.

- [ ] **Step 1: Run fresh build and test verification**

  Run `git diff --check`, full Debug build, focused tests, and Release CLI build. Record any unrelated pre-existing full-suite failures separately.

- [ ] **Step 2: Run 2dot5 and anisotropic parity cases**

  Run both for 20 layers with first height `0.1` and growth ratio `1.2`; compare per-layer cell counts with the accepted runs.

- [ ] **Step 3: Run Benchmark**

  Use the separately scoped temporary orientation preparation, 20 layers, first height `0.1`, growth ratio `1.2`, and maximum skewness `1.0`. Verify layer 20 adds exactly 215,050 cells.

- [ ] **Step 4: Report measured improvement**

  Compare layer-20 `candidate-rejections` against 155.7s, report full versus local build counts and timings, and identify the next largest stage from fresh evidence.
