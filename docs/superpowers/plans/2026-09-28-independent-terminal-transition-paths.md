# Independent Terminal Transition Paths Implementation Plan

> **For agentic workers:** implement inline with TDD checkpoints; keep each checkbox current.

**Goal:** Choose internal versus external terminal-quad paths from aspect ratio, select topology independently within each path, and apply bounded rollback/fallback without rebuilding unaffected patches.

**Architecture:** Keep raw high-neighbor discovery as shared input, but separate internal and external candidate selection. Track attempted paths for one source face; route failed external high-edge patches through resolver rollback so the next build sees the reduced retained set and recomputes adjacency. Treat absent terminal-Hexa geometry explicitly.

**Tech Stack:** C++17, CMake/MSBuild, CTest.

## Global Constraints

- Preserve unrelated user changes already present in the working tree.
- Keep resolver rebuilds local to affected transition patches.
- Maintain deterministic topology selection and existing collision checks.
- Run blades using Release and report completion and elapsed time.

---

### Task 1: Isolate terminal path selection

**Files:**
- Modify: `src/transition/provisional_transition_builder.cpp`
- Test: `tests/unit/transition/quad_high_neighbor_selector_test.cpp` and/or focused transition test target

- [x] Add regression cases for priority-external high-edge failure, no-high external reconstruction, and absent terminal-Hexa geometry.
- [x] Run the focused test and confirm the old behavior failed at the expected rollback assertion (exit 48).
- [x] Route exhausted high-edge external construction through existing forced-rollback provenance; resolver removes those retained high faces and the next build rediscovers zero adjacency before trying external zero-high.
- [x] Verify focused transition tests pass. External patch positivity is already checked inside `buildExternalQuadPatch`; no duplicate volume-evaluation pass was added.

### Task 2: Bound fallback and resolver rollback

**Files:**
- Modify: `src/transition/provisional_transition_builder.cpp`, `src/transition/layer_transition_resolver.cpp` only if required
- Test: `tests/unit/transition/layer_transition_resolver_test.cpp`

- [x] Cover failed external high-edge construction and separately build with the rolled-back retained set to verify the zero-high external patch is selected.
- [x] Confirm the pre-fix behavior failed the new regression.
- [x] Use the resolver's monotonic retained-high-face set as the finite attempt state: a high-edge failure removes its dependency faces once; after no neighbors remain, zero-high external failure falls through to internal and cannot re-enter the high-edge branch.
- [x] Verify resolver fallback terminates through focused unit coverage; only the failed face's high dependencies are requested for rollback.

### Task 3: Verify and benchmark blades

**Files:**
- Modify: `docs/design/modules/layer-transition.md` if behavior documentation needs correction

- [x] Build and run the three focused transition tests in Debug and Release; 3/3 passed in each configuration.
- [x] Build `boundary_mesh_cli` in Release.
- [x] Run blades 20 layers in Release with first height 0.2, growth ratio 1.2, maximum skewness 1; console-visible progress completed all layers, finalization, and VTK writing with exit code 0.
- [x] Elapsed wall time: 1,276.11 s (21 min 16 s). Existing comparable blades run: 1,322.6 s; this single run is ~3.5% lower, too small/uncontrolled to claim a reliable speedup. Layer 20 elapsed 84.049 s; finalization 11.463 s; VTK output 73.787 s.
- [x] Output files: `build/blades_transition_priority_20260928_boundary_layer.vtk` (1,287,106,749 bytes), `_boundary_layer_top.vtk` (515,395,503 bytes), `_farfield_boundary.vtk` (53,821,167 bytes).
- [x] `git diff --check` passed. Existing unrelated working-tree changes were preserved.
