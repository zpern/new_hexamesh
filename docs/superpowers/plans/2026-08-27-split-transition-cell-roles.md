# Split Transition Cell Roles Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Distinguish multi-normal transition cells from reserved-layer transition cells in `CellMetadata`.

**Architecture:** Replace the ambiguous enum value at the shared mesh type, then update each producer and its direct tests. Keep result fields and mesh-generation behavior unchanged.

**Tech Stack:** C++17, CMake, CTest, MSVC Debug/Release.

## Global Constraints

- Delete `CellRole::Transition`; do not retain a compatibility value.
- Do not change geometry generation, ordering, or ID mappings.
- Preserve unrelated working-tree changes.

---

### Task 1: Encode the required role distinction in tests

**Files:**
- Modify: `tests/unit/mesh/volume_mesh_test.cpp`
- Modify: `tests/unit/multi_normal/multi_normal_mesh_merge_test.cpp`
- Modify: `tests/unit/transition/triangle_transition_template_test.cpp`
- Modify: `tests/unit/transition/quad_hexa_decomposition_test.cpp`
- Modify: `tests/integration/reserved_layer_transition_pipeline_test.cpp`

- [ ] Replace expected generic roles with the appropriate dedicated role and change the corner integration assertion to expect zero reserved-layer cells while independently requiring multi-normal cells.
- [ ] Build the affected tests and verify compilation fails because the new enum values do not exist.

### Task 2: Update the shared enum and production role assignments

**Files:**
- Modify: `include/boundary_mesh/mesh/mesh_volume.hpp`
- Modify: `src/multi_normal/multi_normal_transition_builder.cpp`
- Modify: `src/transition/triangle_transition_template.cpp`
- Modify: `src/transition/quad_transition_template.cpp`
- Modify: `src/transition/reserved_layer_transition.cpp`
- Modify: `tests/integration/transition/boundary_layer_generation_pipeline_test.cpp`
- Modify: `tests/integration/multi_normal/multi_normal_growth_pipeline_test.cpp`

- [ ] Define exactly the three approved enum values.
- [ ] Assign `MultiNormalTransition` in the multi-normal producer and `ReservedLayerTransition` in reserved templates.
- [ ] Count only `ReservedLayerTransition` in `reserved_transition_cell_count`.
- [ ] Update remaining assertions to require the precise role.
- [ ] Build and run the focused Debug tests; expect all to pass.

### Task 3: Regression verification

- [ ] Search the repository and verify no `CellRole::Transition` reference remains.
- [ ] Build the complete Debug configuration and run all registered tests.
- [ ] Build the complete Release configuration and run all registered tests.
- [ ] Run `git diff --check` and inspect only the intended paths.
