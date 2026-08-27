# Surface Sliding Constraints Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generalize Symmetry-only growth constraints so Wall vertices incident to either Symmetry or Internal regions slide along their corresponding planes while the growth front remains Wall-only.

**Architecture:** Rename the public Symmetry constraint model to Sliding, carry sorted sliding region IDs from `GrowthPatch` into every `GrowthFront`, and let `SlidingConstraintBuilder` resolve both Symmetry and Internal faces by globally unique `region_id`. Preserve the existing projection mathematics and validation thresholds.

**Tech Stack:** C++17, Eigen, CMake, CTest, existing `boundary_mesh::Result` types.

## Global Constraints

- Only Wall faces enter `GrowthPatch::sourceFaceIds()` and `GrowthFront::faces`.
- Symmetry and Internal region IDs are globally unique across the input mesh.
- Every face in one sliding region must be coplanar.
- Do not keep obsolete Symmetry API aliases or invalid compatibility fields.
- Preserve current `SurfaceTopology::vertexFaces()` semantics and all unrelated working-tree changes.

---

### Task 1: Rename the public constraint model to Sliding

**Files:**
- Rename: `include/boundary_mesh/growth/symmetry_constraints.hpp` -> `include/boundary_mesh/growth/sliding_constraints.hpp`
- Rename: `include/boundary_mesh/growth/symmetry_constraint_builder.hpp` -> `include/boundary_mesh/growth/sliding_constraint_builder.hpp`
- Rename: `src/growth/symmetry_constraint_builder.cpp` -> `src/growth/sliding_constraint_builder.cpp`
- Rename: `tests/unit/growth/symmetry_constraints_test.cpp` -> `tests/unit/growth/sliding_constraints_test.cpp`
- Modify: `include/boundary_mesh/growth/growth_direction_error.hpp`
- Modify: `include/boundary_mesh/growth/growth_patch.hpp`
- Modify: `include/boundary_mesh/growth/growth_front.hpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Modify all compile-time consumers found by `rg`.

**Interfaces:**
- Consumes: existing Symmetry constraint API and behavior.
- Produces: `SlidingPlane`, `VertexSlidingConstraint`, `SlidingConstraints`, `SlidingConstraintBuilder`, `SlidingInputMismatch`, `InvalidSlidingSurface`, and `sliding_region_ids`.

- [ ] **Step 1: Rename the unit test and express the desired public API**

Use `git mv` for the test and update includes/usages to:

```cpp
#include <boundary_mesh/growth/sliding_constraint_builder.hpp>

const auto constraints = SlidingConstraintBuilder{}.build(
    mesh, front, evaluation);
const SlidingPlane &plane = constraints.value().planes()[0];
const VertexSlidingConstraint &vertex =
    constraints.value().vertices()[0];
```

Update error assertions to `SlidingInputMismatch` and `InvalidSlidingSurface`, and front fixtures to initialize `sliding_region_ids`.

- [ ] **Step 2: Verify the desired API fails**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_constraints_test`

Expected: configuration/compilation fails because the renamed target, headers, and types do not exist.

- [ ] **Step 3: Rename files and public types mechanically**

Use `git mv` for the two headers and source. Replace names consistently:

```text
SymmetryPlane             -> SlidingPlane
VertexSymmetryConstraint  -> VertexSlidingConstraint
SymmetryConstraints       -> SlidingConstraints
SymmetryConstraintBuilder -> SlidingConstraintBuilder
SymmetryInputMismatch     -> SlidingInputMismatch
InvalidSymmetrySurface    -> InvalidSlidingSurface
symmetry_region_ids       -> sliding_region_ids
```

Update include guards via `#pragma once` paths, friend declarations, method return types, source includes, CMake source paths, target names, and all repository C++ callers. Do not change region selection behavior in this task; it still resolves Symmetry only.

- [ ] **Step 4: Reconfigure and verify the mechanical migration**

Run: `cmake -S . -B build`

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_constraints_test boundary_mesh_growth_patch_test boundary_mesh_growth_front_test boundary_mesh_growth_front_pipeline_test`

Run: `ctest --test-dir build -C Debug -R "^(boundary_mesh_sliding_constraints_test|boundary_mesh_growth_patch_test|boundary_mesh_growth_front_test|boundary_mesh_growth_front_pipeline_test)$" --output-on-failure`

Expected: 4/4 tests pass with unchanged Symmetry behavior.

- [ ] **Step 5: Commit the API migration**

Stage only renamed constraint files, direct callers, CMake entries, and renamed fields, then commit:

```powershell
git commit -m "refactor: generalize symmetry constraints as sliding"
```

### Task 2: Collect Internal regions without adding Internal faces to the front

**Files:**
- Modify: `src/growth/growth_patch_builder.cpp`
- Modify: `src/growth/growth_front_builder.cpp` only if field migration was incomplete.
- Modify: `tests/unit/growth/growth_patch_test.cpp`
- Modify: `tests/unit/growth/growth_front_test.cpp`

**Interfaces:**
- Consumes: `PatchVertex::sliding_region_ids`, `FrontVertexBoundary::sliding_region_ids`, and complete `SurfaceTopology::vertexFaces()`.
- Produces: sorted unique Symmetry/Internal region inheritance on Wall vertices with Wall-only source faces.

- [ ] **Step 1: Add a failing mixed-boundary patch test**

Construct a closed non-internal shell containing Wall and Symmetry faces plus an attached Internal triangle. Assert exact behavior:

```cpp
assert(patch.value().sourceFaceIds() ==
       std::vector<SurfaceFaceId>{wall_face_id});
assert(patch_vertex.sliding_region_ids ==
       std::vector<std::uint32_t>{symmetry_region, internal_region});
assert(std::find(
           patch.value().sourceFaceIds().begin(),
           patch.value().sourceFaceIds().end(),
           internal_face_id) == patch.value().sourceFaceIds().end());
```

Use duplicate incident faces from the same region to prove sorting/deduplication.

- [ ] **Step 2: Verify the test fails for missing Internal inheritance**

Run: `cmake --build build --config Debug --target boundary_mesh_growth_patch_test`

Run: `ctest --test-dir build -C Debug -R "^boundary_mesh_growth_patch_test$" --output-on-failure`

Expected: test fails because only Symmetry regions are collected.

- [ ] **Step 3: Generalize region collection minimally**

Replace the selection condition with a focused helper or expression:

```cpp
const bool is_sliding =
    tag.kind == SurfaceBoundaryKind::Symmetry ||
    tag.kind == SurfaceBoundaryKind::Internal;
if (is_sliding)
    sliding_region_ids.push_back(tag.region_id);
```

Keep the earlier Wall-only face scan unchanged. Continue sorting and uniquing by numeric region ID.

- [ ] **Step 4: Verify patch/front behavior**

Run: `cmake --build build --config Debug --target boundary_mesh_growth_patch_test boundary_mesh_growth_front_test`

Run: `ctest --test-dir build -C Debug -R "^(boundary_mesh_growth_patch_test|boundary_mesh_growth_front_test)$" --output-on-failure`

Expected: 2/2 tests pass; Internal face IDs are absent from both patch and front.

- [ ] **Step 5: Commit region inheritance**

```powershell
git add -- src/growth/growth_patch_builder.cpp tests/unit/growth/growth_patch_test.cpp tests/unit/growth/growth_front_test.cpp
git commit -m "feat: inherit internal sliding regions"
```

### Task 3: Resolve Internal regions as sliding planes

**Files:**
- Modify: `src/growth/sliding_constraint_builder.cpp`
- Modify: `tests/unit/growth/sliding_constraints_test.cpp`
- Modify: `tests/integration/growth_front_pipeline_test.cpp`

**Interfaces:**
- Consumes: front `sliding_region_ids` from Task 2 and globally unique boundary `region_id`.
- Produces: planar sliding constraints sourced from either Symmetry or Internal faces.

- [ ] **Step 1: Add failing Internal-plane behavior tests**

Add cases using real mesh faces and the existing evaluator:

```text
one Internal region -> direction projected into its plane
one Internal plus one Symmetry region -> direction along their intersection
same Internal region with a displaced non-coplanar face -> InvalidSlidingSurface
missing region -> SlidingInputMismatch
three independent mixed sliding regions -> OverConstrainedGrowthVertex
```

For the one-plane case, use an Internal plane with unit normal `(0,0,1)` and raw direction `(1,1,1)`; assert the normalized result is `(1,1,0)/sqrt(2)` within the existing tolerance.

- [ ] **Step 2: Verify Internal region lookup fails**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_constraints_test`

Run: `ctest --test-dir build -C Debug -R "^boundary_mesh_sliding_constraints_test$" --output-on-failure`

Expected: new Internal cases fail with `SlidingInputMismatch` because lookup still accepts Symmetry only.

- [ ] **Step 3: Generalize plane lookup**

In the region face scan use:

```cpp
const bool is_sliding_kind =
    tag.kind == SurfaceBoundaryKind::Symmetry ||
    tag.kind == SurfaceBoundaryKind::Internal;
if (is_sliding_kind && tag.region_id == region_id)
    region_faces.push_back(face_index);
```

Keep reference-face selection, face evaluation, coplanarity validation, Gram-Schmidt independence checks, projection, direction tolerances, sorting, and error ordering unchanged.

- [ ] **Step 4: Verify unit and pipeline behavior**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_constraints_test boundary_mesh_growth_front_pipeline_test`

Run: `ctest --test-dir build -C Debug -R "^(boundary_mesh_sliding_constraints_test|boundary_mesh_growth_front_pipeline_test)$" --output-on-failure`

Expected: 2/2 tests pass, including Internal constraints on later growth layers.

- [ ] **Step 5: Commit sliding-plane behavior**

```powershell
git add -- src/growth/sliding_constraint_builder.cpp tests/unit/growth/sliding_constraints_test.cpp tests/integration/growth_front_pipeline_test.cpp
git commit -m "feat: constrain growth along internal planes"
```

### Task 4: Repository-wide migration and verification

**Files:**
- Modify direct C++ or current module documentation references missed by earlier tasks.
- Do not rewrite historical plan/spec documents that intentionally describe the old API at that point in time.

**Interfaces:**
- Consumes: complete Sliding API and behavior.
- Produces: no active build reference to obsolete Symmetry constraint identifiers.

- [ ] **Step 1: Audit active code and build files**

Run:

```powershell
rg -n "SymmetryPlane|VertexSymmetryConstraint|SymmetryConstraints|SymmetryConstraintBuilder|InvalidSymmetrySurface|SymmetryInputMismatch|symmetry_region_ids|symmetry_constraint" CMakeLists.txt include src tests
```

Expected: no matches in active code or CMake files.

- [ ] **Step 2: Build the complete Debug tree**

Run: `cmake --build build --config Debug`

Expected: exit code 0.

- [ ] **Step 3: Run the complete test suite**

Run: `ctest --test-dir build -C Debug --output-on-failure`

Expected: all discovered tests pass.

- [ ] **Step 4: Verify formatting and preserve unrelated work**

Run: `git diff --check`

Run: `git status --short`

Expected: no whitespace errors; pre-existing unrelated changes remain present and are not discarded.

- [ ] **Step 5: Report compatibility**

Report that downstream users must rename constraint headers/types/errors and `symmetry_region_ids` to the Sliding equivalents. Confirm that Wall-only front extraction, projection mathematics, and `SurfaceTopology::vertexFaces()` remain unchanged.
