# Provisional Transition Build Context Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build current/candidate front lookup tables once per resolver invocation and reuse them across every full and external-only provisional transition build.

**Architecture:** Add an immutable `ProvisionalTransitionBuildContext` that borrows the two fronts and owns their face, edge, and vertex lookup tables. Context-taking builder overloads are the implementation path; existing overloads become compatibility wrappers, while production captures one context in both resolver callbacks.

**Tech Stack:** C++17, CMake/CTest, existing `GrowthFront`, transition templates, and `Result` types.

## Global Constraints

- Preserve owned triangles, owner metadata, resolved topology, forced rollbacks, and errors.
- The context is reusable when retained faces, face sets, keep-hexa controls, or distance scales change.
- Rebuild the context when either current or candidate front changes.
- Keep the existing front-taking builder overloads source-compatible.
- Do not cache generated geometry or template decisions.
- Benchmark acceptance remains 20 layers, first height `0.1`, growth ratio `1.2`, maximum skewness `1.0`, and 215,050 cells on layer 20.

---

### Task 1: Reusable lookup context and parity

**Files:**
- Modify: `include/boundary_mesh/transition/provisional_transition_builder.hpp`
- Modify: `src/transition/provisional_transition_builder.cpp`
- Modify: `tests/unit/transition/transition_boundary_checker_test.cpp`

**Interfaces:**
- Produces: `ProvisionalTransitionBuildContext(const GrowthFront &, const GrowthFront &)` with immutable front access and face/edge/vertex lookup methods.
- Produces: context-taking overloads of `buildProvisionalTransition()` and `buildProvisionalExternalPatches()`.
- Preserves: existing front-taking overloads as wrappers.

- [ ] **Step 1: Add a failing context parity test**

  In the existing terminal external-patch fixture, construct:

  ```cpp
  const ProvisionalTransitionBuildContext context{
      terminal_current, terminal_candidate};
  ```

  For scales `0.25`, `0.125`, and `0.0625`, call both the existing wrapper and
  the wished-for context overload. Compare candidate triangles, generated
  points, terminal decisions, topology sizes, and forced rollback IDs. Reuse
  the same context after changing retained IDs and `LayerFaceSets`, verifying
  that unretained owners do not remain.

- [ ] **Step 2: Verify RED**

  Run:

  ```powershell
  cmake --build build --config Debug --target boundary_mesh_transition_boundary_checker_test
  ```

  Expected: compilation fails because `ProvisionalTransitionBuildContext` and
  the context-taking overloads do not exist.

- [ ] **Step 3: Add the immutable context API**

  Declare a class that borrows `current` and `candidate`, owns the following
  maps, and exposes const lookup methods returning pointers or optionals:

  ```cpp
  std::unordered_map<SurfaceFaceId, std::size_t> current_faces_;
  std::unordered_map<SurfaceFaceId, std::size_t> candidate_faces_;
  std::unordered_map<std::uint64_t, std::vector<SurfaceFaceId>>
      current_edge_faces_;
  std::unordered_map<std::uint64_t, std::size_t> current_vertices_;
  std::unordered_map<std::uint64_t, std::size_t> candidate_vertices_;
  ```

  The constructor reproduces the current map population order and overwrite
  semantics exactly. The class exposes no mutation and stores no retained,
  face-set, control, or generated-geometry state.

- [ ] **Step 4: Route both builders through the context**

  Change `buildProvisionalTransitionImpl()` to consume a context and use its
  cached maps. Add a cached candidate-face lookup for terminal candidates.
  Convert selected external IDs to a sorted local vector or hash set once per
  call rather than using repeated linear membership checks.

  Implement the old overloads as:

  ```cpp
  const ProvisionalTransitionBuildContext context{current, candidate};
  return buildProvisionalTransition(context, retained, face_sets,
      terminal_hexa_points, external_controls, terminal_candidate_faces);
  ```

- [ ] **Step 5: Verify GREEN and transition regressions**

  Run:

  ```powershell
  cmake --build build --config Debug --target boundary_mesh_transition_boundary_checker_test
  ctest --test-dir build -C Debug -R "(transition_boundary_checker|layer_transition_resolver|incremental_layer_transition)" --output-on-failure
  ```

  Expected: build succeeds and all selected tests pass.

- [ ] **Step 6: Commit**

  Commit only the header, implementation, and unit test:

  ```powershell
  git add -- include/boundary_mesh/transition/provisional_transition_builder.hpp src/transition/provisional_transition_builder.cpp tests/unit/transition/transition_boundary_checker_test.cpp
  git commit -m "perf: cache provisional transition lookups"
  ```

### Task 2: Production context reuse

**Files:**
- Modify: `src/boundary_layer/incremental_boundary_layer_generator.cpp`
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `tests/integration/incremental_layer_transition_pipeline_test.cpp`

**Interfaces:**
- Consumes: `ProvisionalTransitionBuildContext` and its context-taking builder overloads from Task 1.
- Produces: one lookup-context construction per candidate-rejection resolver invocation, shared by both production callbacks.

- [ ] **Step 1: Add a failing production-path assertion**

  In `incremental_layer_transition_pipeline_test.cpp`, include `<iostream>`
  and `<sstream>`. Redirect `std::cerr.rdbuf()` around the existing
  `rollback_surface` call to `generateIncrementalBoundaryLayers()`, restore it
  immediately afterward, and assert that its captured diagnostics contain
  exactly one occurrence of:

  ```text
  temporary resolver lookup_context_builds=1
  ```

  This fixture executes one candidate-rejection resolver invocation. The
  assertion therefore records the production lifetime boundary without adding
  a test-only API to the context.

- [ ] **Step 2: Verify RED**

  Run:

  ```powershell
  cmake --build build --config Debug --target boundary_mesh_incremental_layer_transition_pipeline_test
  ```

  Expected: the new assertion fails because production does not yet construct
  or report a shared lookup context.

- [ ] **Step 3: Construct and capture one context**

  Immediately before assigning `input.build_provisional`, construct:

  ```cpp
  const ProvisionalTransitionBuildContext transition_build_context{
      effective_current, candidate.next_front};
  ```

  Capture it by reference in `build_provisional` and
  `build_external_patches`. Replace both calls with their context-taking
  overloads. Keep the context in scope through `resolve(input)`.

- [ ] **Step 4: Report lookup preparation separately**

  Time context construction in the generator and add one concise diagnostic
  immediately after construction:

  ```text
  temporary resolver lookup_context_builds=1 lookup_context_ms=<N>
  ```

  The literal build count is one because this is the single construction site
  for that resolver invocation. Do not add mutable state or a global counter to
  the context. The diagnostic is observational and must not change builder
  behavior.

- [ ] **Step 5: Run focused verification**

  Run:

  ```powershell
  cmake --build build --config Debug --target boundary_mesh_incremental_layer_transition_pipeline_test boundary_mesh_layer_transition_resolver_test
  ctest --test-dir build -C Debug -R "(regular_layer|transition|collision)" --output-on-failure
  git diff --check
  ```

  Expected: all selected tests pass and `git diff --check` reports no errors.

- [ ] **Step 6: Commit**

  Commit only the production wiring, diagnostics, and integration test:

  ```powershell
  git add -- src/boundary_layer/incremental_boundary_layer_generator.cpp src/transition/layer_transition_resolver.cpp tests/integration/incremental_layer_transition_pipeline_test.cpp
  git commit -m "perf: reuse transition lookup context in probes"
  ```

### Task 3: Release Benchmark verification

**Files:**
- No tracked production changes.
- Store logs and VTK output under the existing Benchmark case directory.

**Interfaces:**
- Consumes: Release CLI with reusable transition context.
- Produces: topology parity, cell-count parity, and updated local-build timings.

- [ ] **Step 1: Build the Release CLI**

  Run:

  ```powershell
  cmake --build build --config Release --target boundary_mesh_cli
  ```

  Expected: exit code 0.

- [ ] **Step 2: Run Benchmark with temporary orientation preparation**

  Temporarily call the already-tested `unifySurfaceOrientation()` helper only
  in the benchmark CLI build, run Benchmark with `0.1`, `1.2`, 20 layers, and
  maximum skewness `1.0`, then remove the temporary call and rebuild the
  official Release CLI.

- [ ] **Step 3: Check parity and timings**

  Require:

  ```text
  finish 20 boundarylayer. add 215050 cell.
  ```

  Compare layer 19 local-build time with `17.192s` and layer 20 with `8.885s`.
  Report context preparation time, local-build time, total
  `candidate-rejections`, and the next largest measured substage.

- [ ] **Step 4: Final verification**

  Run the focused Debug test selection again, rebuild the official Release
  CLI without temporary orientation code, confirm `git diff --check`, and
  verify that only the user's pre-existing uncommitted files remain.
