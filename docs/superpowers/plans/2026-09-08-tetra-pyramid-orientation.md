# Tetra/Pyramid Orientation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every valid transition Tetra and Pyramid use one explicit topology-derived positive orientation while preserving genuine reversed, locally inverted, and degenerate geometry for diagnosis.

**Architecture:** Define the Tetra/Pyramid vertex contracts beside the volume cell types and provide read-only evaluators using fixed subtetrahedra. Update every transition template to construct that contract directly; remove coordinate-sign-driven connectivity mutation. Legacy VTK exports the stored canonical order unchanged and is covered by a connectivity regression test.

**Tech Stack:** C++17, Eigen point/vector types, CMake/CTest, Legacy VTK ASCII.

## Global Constraints

- Never change connectivity based on the computed final volume sign.
- Normal geometry must be positive by construction; negative, mixed-sign, and zero subtetrahedra remain visible as reversed, locally inverted, and degenerate states.
- Prism/Hexa ordering and evaluation remain unchanged.
- There is no CGNS volume writer in the current repository; do not invent one in this change.

---

### Task 1: Define and test the common Tetra/Pyramid orientation contract

**Files:**
- Modify: `include/boundary_mesh/mesh/mesh_volume.hpp`
- Modify: `include/boundary_mesh/quality/volume_cell_evaluation.hpp`
- Modify: `include/boundary_mesh/quality/volume_cell_evaluator.hpp`
- Create: `src/quality/tetra_pyramid_evaluator.cpp`
- Modify: `CMakeLists.txt`
- Create: `tests/unit/quality/tetra_pyramid_orientation_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `evaluateTetra(const TetraPoints&)` and `evaluatePyramid(const PyramidPoints&)`, returning `Result<VolumeCellEvaluation, VolumeCellEvaluationError>`.
- Contract: Tetra uses fixed subtet `{0,1,2,3}`; Pyramid uses fixed subtets `{0,1,2,4}` and `{0,2,3,4}`.

- [ ] **Step 1: Write the failing evaluator test**

Create standard positive Tetra and square-base Pyramid coordinates, reversed connectivity variants, a Pyramid with one apex/diagonal configuration producing mixed signs, and zero-height degenerate variants. Assert `Valid`, `Reversed`, `LocallyInverted`, and `Degenerate` without modifying input order.

- [ ] **Step 2: Run the new test and verify RED**

Run: `cmake --build build --target boundary_mesh_tetra_pyramid_orientation_test && ctest --test-dir build -R tetra_pyramid_orientation --output-on-failure`

Expected: build fails because the new evaluator declarations/definitions do not yet exist.

- [ ] **Step 3: Implement the minimal read-only evaluators**

Add `TetraPoints`/`PyramidPoints` aliases and comments defining the exact vertex contract. Reuse `quality_internal::SubtetVolumeAccumulator`; do not add swapping or normalization. For Pyramid, accumulate the two fixed subtets in the declared order.

- [ ] **Step 4: Verify GREEN**

Run the command from Step 2. Expected: one test passes with zero failures.

- [ ] **Step 5: Commit**

Run: `git add include/boundary_mesh/mesh/mesh_volume.hpp include/boundary_mesh/quality/volume_cell_evaluation.hpp include/boundary_mesh/quality/volume_cell_evaluator.hpp src/quality/tetra_pyramid_evaluator.cpp CMakeLists.txt tests/unit/quality/tetra_pyramid_orientation_test.cpp tests/CMakeLists.txt && git commit -m "test: define tetra pyramid orientation contract"`

### Task 2: Make layer-transition templates obey the fixed contract

**Files:**
- Modify: `tests/unit/transition/triangle_side_transition_test.cpp`
- Modify: `tests/unit/transition/incremental_transition_templates_test.cpp`
- Modify: `src/transition/triangle_side_transition.cpp`
- Modify: `src/transition/incremental_transition_templates.cpp`

**Interfaces:**
- Consumes: `evaluateTetra` and `evaluatePyramid` from Task 1.
- Produces: all cells returned by the four layer-transition builders are `Valid` for their normal geometric fixtures.

- [ ] **Step 1: Extend tests with geometric orientation assertions**

For every output cell, resolve IDs against fixture points, call the matching evaluator, and assert `validity == VolumeCellValidity::Valid` and `signed_volume > 0`. Exercise every high-edge index and both quad diagonals so rotational branches are covered.

- [ ] **Step 2: Run and verify RED**

Run: `cmake --build build --target boundary_mesh_triangle_side_transition_test boundary_mesh_incremental_transition_templates_test && ctest --test-dir build -R "triangle_side_transition|incremental_transition_templates" --output-on-failure`

Expected: at least one existing Tetra/Pyramid template reports a negative or mixed-sign fixed subtet.

- [ ] **Step 3: Correct template connectivity from topology**

Change only the literal/permuted node order in each failing template. Preserve each cell's vertex set, shared faces, `top_faces`, metadata, and diagonals. Determine the permutation from the oriented low/top faces; do not call an evaluator or inspect coordinates in production template construction.

- [ ] **Step 4: Verify GREEN and topology regressions**

Run the command from Step 2, then run: `ctest --test-dir build -R "incremental_layer_transition_pipeline|transition_topology" --output-on-failure`

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

Run: `git add tests/unit/transition src/transition && git commit -m "fix: unify layer transition cell orientation"`

### Task 3: Remove sign-based mutation from multi-normal Tetra generation

**Files:**
- Modify: `tests/unit/multi_normal/multi_normal_transition_builder_test.cpp`
- Modify: `src/multi_normal/multi_normal_transition_builder.cpp`

**Interfaces:**
- Consumes: `evaluateTetra` from Task 1 and the input triangle's local winding.
- Produces: deterministic Tetra connectivity derived only from lower/upper topology.

- [ ] **Step 1: Add regression tests for connectivity and validity**

Add fixtures for collapsed-bottom, distinct-bottom, and repeated-bottom branches. Assert exact expected IDs for each Tetra and positive validity for normal geometry. Add a deliberately reversed geometric fixture and assert the same topology-derived connectivity remains reversed rather than being swapped positive.

- [ ] **Step 2: Run and verify RED**

Run: `cmake --build build --target boundary_mesh_multi_normal_transition_builder_test && ctest --test-dir build -R multi_normal_transition_builder --output-on-failure`

Expected: the reversed fixture exposes that `addTet` currently swaps IDs `1/2`, and one or more exact-connectivity assertions fail.

- [ ] **Step 3: Implement topology-only ordering**

Remove `if (volume < 0) std::swap(ids[1], ids[2]);`. Correct each `addTet({...})` argument order so normal fixtures follow the common Tetra contract. Retain distinct-ID, non-finite, and zero-volume rejection; negative finite volume must not trigger mutation or silent omission.

- [ ] **Step 4: Verify GREEN**

Run the command from Step 2 and: `ctest --test-dir build -R multi_normal --output-on-failure`.

Expected: all multi-normal tests pass, including the deliberate reversed-geometry diagnostic.

- [ ] **Step 5: Commit**

Run: `git add tests/unit/multi_normal/multi_normal_transition_builder_test.cpp src/multi_normal/multi_normal_transition_builder.cpp && git commit -m "fix: derive multi normal tetra winding from topology"`

### Task 4: Verify Legacy VTK preserves canonical connectivity

**Files:**
- Modify: `tests/unit/io/legacy_vtk_writer_test.cpp`
- Modify only if a fixed format mapping is proven necessary: `src/io/legacy_vtk_writer.cpp`

**Interfaces:**
- Consumes: canonical Tetra/Pyramid order.
- Produces: VTK Tetra type 10 and Pyramid type 14 connectivity in the same fixed order, with no geometry-dependent remapping.

- [ ] **Step 1: Add an exported-connectivity regression test**

Write one positive canonical Tetra and Pyramid, parse the `CELLS` section, assert exact connectivity, reconstruct coordinates, and assert positive evaluator results.

- [ ] **Step 2: Run the test**

Run: `cmake --build build --target boundary_mesh_legacy_vtk_writer_test && ctest --test-dir build -R legacy_vtk_writer --output-on-failure`.

Expected: PASS if the existing direct output already matches VTK; otherwise FAIL identifies a fixed mapping mismatch.

- [ ] **Step 3: Apply only a fixed standards mapping if required**

If Step 2 proves a mismatch, replace direct iteration for the affected type with a compile-time index permutation. The mapping must depend only on cell type, never coordinates or volume sign. If Step 2 passes, make no production writer change.

- [ ] **Step 4: Verify GREEN**

Re-run Step 2. Expected: all writer tests pass.

- [ ] **Step 5: Commit**

Run: `git add tests/unit/io/legacy_vtk_writer_test.cpp src/io/legacy_vtk_writer.cpp && git commit -m "test: verify vtk tetra pyramid orientation"`

### Task 5: Full verification and documentation alignment

**Files:**
- Modify: `README.md`

**Interfaces:**
- Produces: documented node contracts and fresh complete build/test evidence.

- [ ] **Step 1: Document the two vertex orders**

Extend the `VolumeMesh` section next to `PrismVertexOrder`/`HexaVertexOrder` with the Tetra and Pyramid base/apex semantics and state that negative volume is diagnostic, not automatically corrected.

- [ ] **Step 2: Build all configured targets**

Run: `cmake --build build --config Debug`

Expected: exit code 0.

- [ ] **Step 3: Run the complete suite**

Run: `ctest --test-dir build -C Debug --output-on-failure`

Expected: 100% tests passed, zero failures.

- [ ] **Step 4: Inspect the final diff**

Run: `git diff HEAD~4 --check && git status --short --ignore-submodules=all`

Expected: no whitespace errors; only planned files or pre-existing user changes are present.

- [ ] **Step 5: Commit documentation**

Run: `git add README.md && git commit -m "docs: describe tetra pyramid vertex winding"`
