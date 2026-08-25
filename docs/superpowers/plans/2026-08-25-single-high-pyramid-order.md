# Single-high-edge Pyramid Ordering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make both single-high-edge quad transition pyramids use cyclic VTK base-vertex order.

**Architecture:** Correct connectivity in `appendSideTransition`, where the transition cells originate. Keep the VTK writer, tetrahedra, top triangles, and double-high templates unchanged.

**Tech Stack:** C++17, CMake, CTest, legacy VTK output.

## Global Constraints

- Work only on `codex/reserved-layer-transition`; do not modify `master`.
- Preserve cell counts and authored top-face connectivity.
- Use a failing focused regression test before changing production code.
- Regenerate the recovered 2dot5 case with the specified 20-layer parameters.

---

### Task 1: Correct single-high pyramid connectivity

**Files:**
- Modify: `tests/unit/transition/quad_side_transition_template_test.cpp`
- Modify: `src/transition/quad_transition_template.cpp`

**Interfaces:**
- Consumes: `buildQuadTransition(const QuadTransitionInput&)`.
- Produces: `SourceTransitionResult` whose single-high pyramids have cyclic first-four vertex order.

- [ ] **Step 1: Write the failing tests**

Change the edge-0 expected pyramid to:

```cpp
assert((std::get<Pyramid>(result.value().side_cells[0]).vertex_ids ==
        std::array<VertexId,5>{8,9,5,4,6}));
```

Add an edge-1 case, which exercises the opposite initializer branch for the
selected `ZeroTwo` diagonal, and assert:

```cpp
auto alternate = input;
alternate.high_edge_local_index = 1;
const auto alternate_result = buildQuadTransition(alternate);
assert(alternate_result.hasValue());
assert((std::get<Pyramid>(alternate_result.value().side_cells[0]).vertex_ids ==
        std::array<VertexId,5>{10,9,5,6,4}));
```

- [ ] **Step 2: Verify RED**

Run:

```powershell
cmake --build build-cgns --config Debug --target quad_side_transition_template_test
ctest --test-dir build-cgns -C Debug -R '^quad_side_transition_template_test$' --output-on-failure
```

Expected: the focused test fails because production still returns the crossing order.

- [ ] **Step 3: Apply the minimal production fix**

Change only the two single-high pyramid initializers:

```cpp
pyramid = Pyramid{{e, f, b, a, d}};
```

and:

```cpp
pyramid = Pyramid{{f, e, a, b, c}};
```

- [ ] **Step 4: Verify GREEN and regression suite**

Run the focused test again, then:

```powershell
ctest --test-dir build-cgns -C Debug --output-on-failure
```

Expected: focused test and all Debug tests pass.

- [ ] **Step 5: Commit the code and regression test**

```powershell
git add -- src/transition/quad_transition_template.cpp tests/unit/transition/quad_side_transition_template_test.cpp
git commit -m "fix: order single-high pyramid bases cyclically"
```

### Task 2: Verify the recovered real case

**Files:**
- Read: `real-runs/recovered-2dot5-3168674/input.cgns`
- Generate: `real-runs/recovered-2dot5-cyclic/2dot5_boundary_layer.vtk`
- Generate: `real-runs/recovered-2dot5-cyclic/2dot5_boundary_layer_top.vtk`

**Interfaces:**
- Consumes: `boundary_mesh_cli.exe` and the recovered CGNS plus its boundary map.
- Produces: regenerated VTK output with corrected pyramid connectivity.

- [ ] **Step 1: Build the Release CLI**

```powershell
cmake --build build-cgns --config Release --target boundary_mesh_cli
```

Expected: build exits with code 0.

- [ ] **Step 2: Run the real case**

Run with `--first-height 0.1 --growth-ratio 1.2 --layer-count 20 --maximum-skewness 1 --isotropic-height 1.0` and output prefix `real-runs/recovered-2dot5-cyclic/2dot5`.

Expected: all 22 trial layers finish and the process exits with code 0.

- [ ] **Step 3: Inspect the regression cell and top types**

Parse cell 153585 from the regenerated volume VTK and assert its connectivity is a five-node pyramid with a cyclic quadrilateral base. Parse `CELL_TYPES` in the top VTK and assert every declared type is `5`.

Expected: no crossing base edges and zero non-triangle top cells.
