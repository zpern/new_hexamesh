# Double-High Tetra Skewness Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Select the two tetrahedra in a double-high-edge quad transition by minimizing the maximum existing equiangular face skewness while preserving the forced diagonal and both pyramids.

**Architecture:** Keep the change local to the quad transition template. Add private helpers that score a tetrahedron from its four triangular faces and score a two-tetra candidate by its worst tetrahedron, then use that selector only inside `appendDoubleSideTransition`.

**Tech Stack:** C++17, existing `Point3`, `Tetra`, `Result`, and `triangleEquiangularSkewness` APIs; CMake/CTest with MSVC Release builds.

## Global Constraints

- Preserve pyramids `abfed` and `aegcd` exactly.
- Preserve the forced common-corner-to-opposite-corner diagonal `a-d`.
- Method one is `egfd` plus `gfhd`; method two is `eghd` plus `efhd`.
- Each tetrahedron skewness is the maximum `triangleEquiangularSkewness` of its four faces.
- A candidate score is the maximum skewness of its two tetrahedra.
- Select method two on an exact score tie.
- If only one candidate can be evaluated, select it; fail the template if neither can be evaluated.

---

### Task 1: Lock the selection behavior with a failing unit test

**Files:**
- Modify: `tests/unit/transition/quad_side_transition_template_test.cpp`

**Interfaces:**
- Consumes: `buildQuadTransition(const QuadTransitionInput&)`
- Produces: regression assertions on the two tetrahedra emitted after the unchanged two pyramids

- [ ] **Step 1: Add a method-one-wins geometry test**

Construct a double-high transition whose high-layer points reproduce the observed mesh geometry where method one has a smaller worst face skewness. Assert that `side_cells[0]` and `[1]` remain the existing pyramids and that `side_cells[2]` and `[3]` are `Tetra{{e,g,f,d}}` and `Tetra{{g,f,h,d}}`.

- [ ] **Step 2: Add method-two and tie coverage**

Retain the existing regular cube assertion as the deterministic tie/method-two expectation, and add a non-symmetric geometry if necessary to prove method two wins independently of the tie rule.

- [ ] **Step 3: Run the focused test and verify RED**

Run:

```powershell
cmake --build build-cgns --config Release --target boundary_mesh_quad_side_transition_template_test
ctest --test-dir build-cgns -C Release -R boundary_mesh_quad_side_transition_template_test --output-on-failure
```

Expected: the method-one-wins assertion fails because production code always emits method two.

### Task 2: Implement candidate scoring and selection

**Files:**
- Modify: `src/transition/quad_transition_template.cpp`

**Interfaces:**
- Consumes: `triangleEquiangularSkewness(const std::array<Point3,3>&, Scalar)` and the template's mesh vertex array
- Produces: a private selector returning one complete two-tetra candidate or a `FaceEvaluationError`

- [ ] **Step 1: Add tetrahedron face-skewness scoring**

For tetrahedron vertex IDs `{p0,p1,p2,p3}`, evaluate faces `{p0,p1,p2}`, `{p0,p3,p1}`, `{p1,p3,p2}`, and `{p2,p3,p0}`. Return the maximum of the four existing triangle skewness values.

- [ ] **Step 2: Add two-candidate comparison**

Build:

```cpp
method_one = {Tetra{{e,g,f,d}}, Tetra{{g,f,h,d}}};
method_two = {Tetra{{e,g,h,d}}, Tetra{{e,f,h,d}}};
```

Score both pairs, choose the smaller valid score, and choose method two on equality. If one score fails, use the other; if both fail, propagate an evaluation error.

- [ ] **Step 3: Make double-high generation report selection failure**

Change `appendDoubleSideTransition` to return a result, pass `mesh_vertices` and `length_tolerance` through the existing input, append the unchanged pyramids followed by the selected tetrahedra, and make both call sites in `buildQuadTransition` convert a failed selection into `TransitionTemplateError`.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the Task 1 commands. Expected: the focused test passes.

- [ ] **Step 5: Run transition regression tests**

Run:

```powershell
ctest --test-dir build-cgns -C Release -R "boundary_mesh_(quad_diagonal|quad_side_transition_template|transition_layer_coordinator|reserved_layer_transition).*" --output-on-failure
```

Expected: all selected transition tests pass.

### Task 3: Build and verify the main Release executable

**Files:**
- Verify: `build-cgns/Release/boundary_mesh_cli.exe`

**Interfaces:**
- Consumes: the updated transition library
- Produces: a freshly linked Release CLI

- [ ] **Step 1: Build the Release CLI**

```powershell
cmake --build build-cgns --config Release --target boundary_mesh_cli
```

Expected: exit code 0 and `boundary_mesh_cli.exe` is linked.

- [ ] **Step 2: Run the full test suite**

```powershell
ctest --test-dir build-cgns -C Release --output-on-failure
```

Expected: zero failed tests.

- [ ] **Step 3: Check the worktree and commit implementation**

```powershell
git diff --check
git status --short
git add -- src/transition/quad_transition_template.cpp tests/unit/transition/quad_side_transition_template_test.cpp docs/superpowers/plans/2026-08-25-double-high-tetra-skewness.md
git commit -m "fix: minimize double-high tetra skewness"
```

Do not add or modify the existing untracked `.superpowers/` directory.
