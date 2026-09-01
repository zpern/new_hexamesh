# Zero-layer Corner Lowering Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent zero-layer stopped faces from lowering non-contact-corner faces while preserving all positive-layer behavior and the existing shared-edge propagation implementation.

**Architecture:** Add one early guard inside the existing stop-cell loop in `TerminationPropagator::filterSingleHighEdgeCandidates()`. The guard applies only to high-edge/corner processing; `TerminationPropagator::propagate()` is unchanged.

**Tech Stack:** C++17, CMake, CTest, assert-based unit tests.

## Global Constraints

- A stopped face with `completed_layer == 0` must not select a high edge or constrain `non_contact_corner_faces`.
- Shared-edge layer-difference propagation must remain unchanged.
- A stopped face with `completed_layer > 0` must retain the existing behavior.

---

### Task 1: Add and implement the zero-layer guard

**Files:**
- Modify: `tests/unit/growth/termination_propagator_test.cpp`
- Modify: `src/growth/termination_propagator.cpp`

**Interfaces:**
- Consumes: `TerminationPropagator::filterSingleHighEdgeCandidates(const GrowthFront &, const LayerStepResult &, FaceLayerConstraintTable &, std::uint32_t, std::vector<SurfaceFaceId> &) const`
- Produces: unchanged public interface with corrected zero-layer behavior.

- [ ] **Step 1: Write the failing regression test**

After the existing `actual_filtered` assertions, reuse the same three-face patch and build a first-layer candidate result whose first source face is absent:

```cpp
    LayerStepResult zero_layer_candidates = actual_candidates;
    zero_layer_candidates.layer = 1;
    zero_layer_candidates.next_front.layer = 1;
    zero_layer_candidates.stopped_faces = {{
        0, 1, 1, FaceStopReason::Collision}};
    auto zero_layer_constraints = initial.value();
    std::vector<SurfaceFaceId> zero_layer_pending;
    const auto zero_layer_filtered =
        propagator.value().filterSingleHighEdgeCandidates(
            front.value(),
            zero_layer_candidates,
            zero_layer_constraints,
            1,
            zero_layer_pending);
    assert(zero_layer_filtered.hasValue());
    assert(zero_layer_constraints.find(4)->allowed_layer_count == 10);
    assert((zero_layer_filtered.value().next_front.source_face_ids ==
            std::vector<SurfaceFaceId>{2, 4}));
```

- [ ] **Step 2: Build and run the focused test to verify RED**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_termination_propagator_test
.\build\tests\Release\boundary_mesh_termination_propagator_test.exe
```

Expected: the executable aborts on the new assertion because current code lowers source face `4` from 10 to 0.

- [ ] **Step 3: Add the minimal production guard**

Immediately after obtaining `completed_layer` in the stop-cell loop, add:

```cpp
            if (completed_layer == 0)
                continue;
```

This skips only the subsequent high-edge selection and non-contact-corner loop.

- [ ] **Step 4: Build and verify GREEN**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_termination_propagator_test
.\build\tests\Release\boundary_mesh_termination_propagator_test.exe
ctest --test-dir build -C Release --output-on-failure
```

Expected: the focused executable exits 0 and all CTest tests pass.

- [ ] **Step 5: Review the exact change**

Run:

```powershell
git diff --check
git diff -- src/growth/termination_propagator.cpp tests/unit/growth/termination_propagator_test.cpp
git status --short
```

Expected: only the regression test and the zero-layer guard are uncommitted implementation changes; the plan remains committed separately.
