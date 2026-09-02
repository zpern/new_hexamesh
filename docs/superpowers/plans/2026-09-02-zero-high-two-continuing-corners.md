# Zero-High Two-Continuing-Corners Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add conforming Triangle and Quad transition templates for a face with no high-neighbor edge and exactly two adjacent continuing corners.

**Architecture:** Keep edge-height and corner-continuation semantics separate. Coordination records a dedicated `continuing_edge_local_index`; Triangle and Quad template inputs consume it through new mutually exclusive branches, while existing high-edge branches remain unchanged. Quad reuses the existing diagonal and tetra-pair skewness helpers.

**Tech Stack:** C++17, CMake, CTest, existing `Result`, transition templates, equiangular face skewness.

## Global Constraints

- Work directly in the current checkout as explicitly requested by the user.
- Preserve all pre-existing staged and unstaged changes.
- `trial_layers == 0` never records or accepts a continuing edge.
- `continuing_edge_local_index` is distinct from and mutually exclusive with all high-edge state.
- Existing high-edge and two-reserved-layer behavior must remain compatible.
- Use test-first red/green cycles for each production behavior.

---

### Task 1: Triangle continuing-edge template

**Files:**
- Modify: `include/boundary_mesh/transition/transition_templates.hpp`
- Modify: `src/transition/triangle_transition_template.cpp`
- Test: `tests/unit/transition/triangle_transition_template_test.cpp`

**Interfaces:**
- Produces: `TriangleTransitionInput::continuing_edge_local_index` as `std::optional<std::size_t>`.
- Consumes: existing `layer_vertex_ids`, `trial_layers`, and `high_edge_local_index`.

- [ ] **Step 1: Write failing Triangle template tests**

Add cases for `trial_layers == 1`, continuing edge 0, and all three rotations. Assert edge 0 produces `Pyramid{{0,3,4,1,2}}`, four exposed top faces, one transition metadata record, and no Prism. Add invalid-input assertions for continuing edge plus high edge, local edge 3, and trial 0.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cmake --build build --config Release --target boundary_mesh_triangle_transition_template_test && .\build\tests\Release\boundary_mesh_triangle_transition_template_test.exe`

Expected: compile failure because `continuing_edge_local_index` does not exist.

- [ ] **Step 3: Implement the minimal Triangle branch**

Add the optional input field and validation. After regular Prism generation, branch on the continuing edge before the ordinary no-high return, append `Pyramid{{low[first], high[first], high[second], low[second], low[apex]}}`, metadata at `occupied + 1`, and its four exposed triangular faces.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the command from Step 2; expect exit code 0.

### Task 2: Quad continuing-edge template and quality choice

**Files:**
- Modify: `include/boundary_mesh/transition/transition_templates.hpp`
- Modify: `src/transition/quad_transition_template.cpp`
- Test: `tests/unit/transition/quad_side_transition_template_test.cpp`

**Interfaces:**
- Produces: `QuadTransitionInput::continuing_edge_local_index` as `std::optional<std::size_t>`.
- Reuses: `selectLayerQuad()`, `tetraPairSkewness()`, regular/base-block diagonal selection, and deterministic score tie behavior.

- [ ] **Step 1: Write failing Quad connectivity tests**

Add a `trial_layers == 1`, edge-0 case. For the bottom `b-c` direction assert the first two cells are `Pyramid{{a,e,f,b,c}}` and `Tetra{{b,c,f,d}}`; assert the selected remaining pair is exactly one of the two approved `e-g`/`f-h` candidates and all four cells carry transition metadata. Add a rotated/mirrored case that asserts `Pyramid{{b,f,e,a,d}}` and `Tetra{{a,d,e,c}}`.

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cmake --build build --config Release --target boundary_mesh_quad_side_transition_template_test && .\build\tests\Release\boundary_mesh_quad_side_transition_template_test.exe`

Expected: compile failure because the new input field does not exist.

- [ ] **Step 3: Add failing quality-selection and validation cases**

Use deterministic point sets to exercise each approved tetra pair. Compute each candidate score in the test with `triangleEquiangularSkewness` over all eight tetra faces, assert the emitted pair has the smaller worst score, and add invalid assertions for simultaneous high/continuing state, edge index 4, and trial 0.

- [ ] **Step 4: Implement the minimal Quad branch**

Validate state exclusivity. Add a focused helper that receives the continuing edge, selected bottom diagonal, and occupied layer; emits the forced Pyramid/first Tetra orientation, evaluates the two remaining tetra pairs, appends the lower-worst-skewness pair, derives the exposed `top_faces`, and appends uniform metadata. Invoke it directly for trial 1 and after the existing base block for trial 2+ without reselecting the diagonal.

- [ ] **Step 5: Run the focused test and verify GREEN**

Run the command from Step 2; expect exit code 0.

### Task 3: Front coordination and pipeline wiring

**Files:**
- Modify: `include/boundary_mesh/transition/transition_coordination.hpp`
- Modify: `src/transition/reserved_layer_transition.cpp`
- Test: `tests/integration/reserved_layer_transition_pipeline_test.cpp`

**Interfaces:**
- Produces: `CoordinatedTransitionFace::continuing_edge_local_index`.
- Consumes: transformed-front `vertex_uses`, per-face `trial_layers`, and high-edge lists.
- Passes the local edge into `TriangleTransitionInput` and `QuadTransitionInput`.

- [ ] **Step 1: Write a failing coordination regression test**

Construct a small connected front whose target face has equal `occupied_layers` on shared edges but exactly two adjacent vertices incident to faces with greater `trial_layers`. Run the reserved transition pipeline and assert the target coordinated face has no `high_edge_local_index` and has the expected `continuing_edge_local_index`. Include Triangle and Quad targets, a Quad with two opposite continuing vertices, and a zero-trial target.

- [ ] **Step 2: Run the focused pipeline test and verify RED**

Run: `cmake --build build --config Release --target boundary_mesh_reserved_layer_transition_pipeline_test && .\build\tests\Release\boundary_mesh_reserved_layer_transition_pipeline_test.exe`

Expected: compile failure for the missing coordinated field or assertion failure because no continuing edge is recorded.

- [ ] **Step 3: Implement common continuing-corner detection**

Add the coordinated field. Extract/reuse a helper that returns true once any other incident face has strictly greater `trial_layers`. Preserve the existing single-high Quad scan. For zero-high faces with `trial_layers >= 1`, scan every local corner, deduplicate by corner, and record an edge only for exactly two adjacent local indices.

- [ ] **Step 4: Wire the state into both template inputs**

Copy `state.continuing_edge_local_index` into Triangle/Quad inputs. Keep source/front local-index mapping validation consistent with the existing third-corner mapping.

- [ ] **Step 5: Run the focused pipeline and template tests and verify GREEN**

Run: `ctest --test-dir build -C Release -R "triangle_transition_template|quad_side_transition_template|reserved_layer_transition_pipeline" --output-on-failure`

Expected: all selected tests pass.

### Task 4: Documentation and full regression verification

**Files:**
- Modify: `docs/reserved_layer_transition.md`

**Interfaces:**
- Documents only the behavior actually passing in Tasks 1-3.

- [ ] **Step 1: Update implemented-rule documentation**

Add the new state to the coordination section, support table, short-layer table, Triangle/Quad template sections, input validation, and quality-selection explanation. Explicitly distinguish continuing edge from high edge and state that trial 0 ignores corner continuation.

- [ ] **Step 2: Build Release**

Run: `cmake --build build --config Release`

Expected: build completes without compiler errors.

- [ ] **Step 3: Run the full test suite**

Run: `ctest --test-dir build -C Release --output-on-failure`

Expected: all registered tests pass.

- [ ] **Step 4: Check patch hygiene**

Run: `git diff --check` and inspect `git diff --stat` plus the relevant file diffs. Expect no whitespace errors and no unrelated files introduced by this feature.
