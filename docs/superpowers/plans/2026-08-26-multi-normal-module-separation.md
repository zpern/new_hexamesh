# Multi-Normal Module Separation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move all multi-normal and BLMesh adapter code out of `growth` into an independently linkable `BoundaryMesh::MultiNormal` module without changing mesh behavior.

**Architecture:** `BoundaryMesh::BoundaryLayer` remains the regular-layer foundation. `BoundaryMesh::MultiNormal` depends on that foundation for `GrowthFront` and front evaluation, while `BoundaryMesh::Transition` owns the combined MultiNormal → Regular → Merge orchestration. This one-way dependency prevents BoundaryLayer from depending on MultiNormal.

**Tech Stack:** C++17, CMake 3.20+, Visual Studio 2022/MSBuild, CTest, Eigen, bundled BLMesh multi-normal sources, CGNS/HDF5.

## Global Constraints

- Do not change split planning, skewness selection, intersection shrinkage, transition-cell construction, CLI options, or output semantics.
- Keep all existing public C++ type names, function names, and the `boundary_mesh` namespace.
- Change public include paths from `boundary_mesh/growth/multi_normal_*.hpp` to `boundary_mesh/multi_normal/multi_normal_*.hpp`.
- `boundary_mesh_boundary_layer` must not compile or link MultiNormal or BLMesh multi-normal implementation code.
- Preserve unrelated user changes and do not modify the main checkout.
- Every production change follows a failing structural test and all 72 baseline tests must remain green.

---

### Task 1: Add a failing standalone-module boundary test

**Files:**
- Create: `tests/cmake/multi_normal_module_boundary_test.cmake`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: repository layout and the root `CMakeLists.txt` as plain text.
- Produces: CTest `boundary_mesh_multi_normal_module_boundary_test`, which enforces the new module paths and CMake target names.

- [ ] **Step 1: Add the structural test script**

Create `tests/cmake/multi_normal_module_boundary_test.cmake` with these checks:

```cmake
if(NOT DEFINED PROJECT_ROOT)
    message(FATAL_ERROR "PROJECT_ROOT is required")
endif()

set(required_paths
    "include/boundary_mesh/multi_normal/multi_normal_types.hpp"
    "include/boundary_mesh/multi_normal/multi_normal_transition_generator.hpp"
    "include/boundary_mesh/multi_normal/incident_face_fan.hpp"
    "include/boundary_mesh/multi_normal/detail/blmesh_geometry.hpp"
    "src/multi_normal/multi_normal_transition_generator.cpp"
    "src/multi_normal/incident_face_fan.cpp"
    "src/multi_normal/blmesh_geometry.cpp")

foreach(relative_path IN LISTS required_paths)
    if(NOT EXISTS "${PROJECT_ROOT}/${relative_path}")
        message(FATAL_ERROR "Missing standalone multi-normal path: ${relative_path}")
    endif()
endforeach()

set(forbidden_paths
    "include/boundary_mesh/growth/multi_normal_types.hpp"
    "include/boundary_mesh/growth/incident_face_fan.hpp"
    "include/boundary_mesh/growth/detail/blmesh_geometry.hpp"
    "src/growth/multi_normal_transition_generator.cpp"
    "src/growth/incident_face_fan.cpp"
    "src/growth/blmesh_geometry.cpp")

foreach(relative_path IN LISTS forbidden_paths)
    if(EXISTS "${PROJECT_ROOT}/${relative_path}")
        message(FATAL_ERROR "Growth still owns multi-normal path: ${relative_path}")
    endif()
endforeach()

file(READ "${PROJECT_ROOT}/CMakeLists.txt" root_cmake)
foreach(required_text IN ITEMS
        "boundary_mesh_multi_normal"
        "BoundaryMesh::MultiNormal")
    string(FIND "${root_cmake}" "${required_text}" position)
    if(position EQUAL -1)
        message(FATAL_ERROR "Root CMake is missing: ${required_text}")
    endif()
endforeach()
```

- [ ] **Step 2: Register the structural test**

Append this test registration to `tests/CMakeLists.txt`:

```cmake
add_test(
    NAME boundary_mesh_multi_normal_module_boundary_test
    COMMAND
        ${CMAKE_COMMAND}
        -DPROJECT_ROOT=${PROJECT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/multi_normal_module_boundary_test.cmake
)
```

- [ ] **Step 3: Reconfigure and verify RED**

Run:

```powershell
cmake -S . -B build
ctest --test-dir build -C Release -R boundary_mesh_multi_normal_module_boundary_test --output-on-failure
```

Expected: the test fails with `Missing standalone multi-normal path: include/boundary_mesh/multi_normal/multi_normal_types.hpp`.

Do not commit the failing state; Task 2 supplies the implementation that makes this test pass.

---

### Task 2: Move MultiNormal code and introduce `BoundaryMesh::MultiNormal`

**Files:**
- Move: `include/boundary_mesh/growth/multi_normal_*.hpp` → `include/boundary_mesh/multi_normal/`
- Move: `include/boundary_mesh/growth/incident_face_fan.hpp` → `include/boundary_mesh/multi_normal/incident_face_fan.hpp`
- Move: `include/boundary_mesh/growth/blmesh_*.hpp` → `include/boundary_mesh/multi_normal/`
- Move: `include/boundary_mesh/growth/detail/blmesh_*.hpp` → `include/boundary_mesh/multi_normal/detail/`
- Move: `src/growth/multi_normal_*.cpp` → `src/multi_normal/`
- Move: `src/growth/incident_face_fan.cpp` → `src/multi_normal/incident_face_fan.cpp`
- Move: `src/growth/blmesh_*.cpp` → `src/multi_normal/`
- Move: `tests/unit/growth/{multi_normal_*,blmesh_*,incident_face_fan_test}.cpp` → `tests/unit/multi_normal/`
- Move: `tests/integration/multi_normal_*.cpp` → `tests/integration/multi_normal/`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Modify: every moved source/header/test and every remaining consumer containing an old MultiNormal include path

**Interfaces:**
- Consumes: `GrowthFront`, front evaluation, Core, Surface, Quality, Spatial, IO, and bundled `third/blmesh_mnormal` sources.
- Produces: CMake target `boundary_mesh_multi_normal`, alias `BoundaryMesh::MultiNormal`, and unchanged APIs under new include paths.

- [ ] **Step 1: Move the tracked files without changing their contents**

Use `git mv` for the complete file sets listed above. Preserve filenames. Place `blmesh_geometry.hpp` and `blmesh_splitter.hpp` in `include/boundary_mesh/multi_normal/detail/`; all other MultiNormal/BLMesh headers go directly in `include/boundary_mesh/multi_normal/`.

- [ ] **Step 2: Rewrite include paths mechanically**

Apply these exact prefix mappings across `include/`, `src/`, and `tests/`:

```text
boundary_mesh/growth/multi_normal_  → boundary_mesh/multi_normal/multi_normal_
boundary_mesh/growth/incident_face_fan.hpp → boundary_mesh/multi_normal/incident_face_fan.hpp
boundary_mesh/growth/blmesh_ → boundary_mesh/multi_normal/blmesh_
boundary_mesh/growth/detail/blmesh_ → boundary_mesh/multi_normal/detail/blmesh_
```

Do not change includes for ordinary growth types such as `growth_front.hpp`, `front_evaluator.hpp`, or `growth_profile.hpp`.

- [ ] **Step 3: Split the root CMake targets**

Keep these regular sources in `boundary_mesh_boundary_layer` and remove every MultiNormal/BLMesh source plus `boundary_layer_generator.cpp` from it:

```cmake
add_library(
    boundary_mesh_boundary_layer
    STATIC
        src/growth/growth_patch_builder.cpp
        src/growth/growth_front_builder.cpp
        src/growth/front_adjacency.cpp
        src/growth/isotropic_stop_evaluator.cpp
        src/growth/front_evaluator.cpp
        src/growth/growth_direction.cpp
        src/growth/growth_field_smoother.cpp
        src/growth/skewness_direction_refiner.cpp
        src/growth/symmetry_constraint_builder.cpp
        src/growth/growth_profile_builder.cpp
        src/growth/regular_layer_stepper.cpp
        src/growth/regular_layer_generator.cpp
        src/growth/exposed_boundary.cpp
        src/growth/farfield_boundary_builder.cpp
        src/growth/layer_collision_checker.cpp
        src/growth/face_layer_constraint.cpp
        src/growth/termination_propagator.cpp
)
```

After defining `BoundaryMesh::BoundaryLayer`, add the new library with this ownership:

```cmake
add_library(
    boundary_mesh_multi_normal
    STATIC
        src/multi_normal/incident_face_fan.cpp
        src/multi_normal/blmesh_splitter.cpp
        src/multi_normal/blmesh_geometry.cpp
        src/multi_normal/blmesh_intersection_checker.cpp
        src/multi_normal/blmesh_topology_stitching.cpp
        third/blmesh_mnormal/src/combineoptimizer.cpp
        third/blmesh_mnormal/src/mergeoptimizer.cpp
        third/blmesh_mnormal/src/meshevaluation.cpp
        third/blmesh_mnormal/src/TopologyOptimizer.cpp
        third/blmesh_mnormal/src/VirtualSphereMeshStrategy.cpp
        third/blmesh_mnormal/src/BoundaryTriangulation.cpp
        third/blmesh_mnormal/src/VirtualSphereMeshHasher.cpp
        third/blmesh_mnormal/src/MeshSplitter.cpp
        third/blmesh_mnormal/src/pointoptimizer.cpp
        third/blmesh_mnormal/src/VirtualSphereMeshGenerator.cpp
        third/blmesh_mnormal/src/VirtualSphereMesh.cpp
        third/blmesh_mnormal/src/BLVector.cpp
        third/blmesh_mnormal/src/sphericalCap.cpp
        third/blmesh_mnormal/src/complexnode.cpp
        third/blmesh_mnormal/src/geometryfunction.cpp
        third/blmesh_mnormal/src/mostnormaloptimizer.cpp
        src/multi_normal/multi_normal_split_planner.cpp
        src/multi_normal/multi_normal_topology_builder.cpp
        src/multi_normal/multi_normal_quad_triangulator.cpp
        src/multi_normal/multi_normal_transition_builder.cpp
        src/multi_normal/multi_normal_intersection_resolver.cpp
        src/multi_normal/multi_normal_transition_generator.cpp
        src/multi_normal/multi_normal_mesh_merge.cpp
)
add_library(BoundaryMesh::MultiNormal ALIAS boundary_mesh_multi_normal)
target_compile_features(boundary_mesh_multi_normal PUBLIC cxx_std_17)
target_include_directories(
    boundary_mesh_multi_normal
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third/blmesh_mnormal/include
        ${PROJECT_SOURCE_DIR}/third/blmesh_mnormal/src
        ${TIGER_DEPENDENCIES_DIR}/geom
)
target_link_libraries(
    boundary_mesh_multi_normal
    PUBLIC
        BoundaryMesh::BoundaryLayer
        BoundaryMesh::Core
        BoundaryMesh::Surface
        BoundaryMesh::Quality
        BoundaryMesh::Spatial
)
```

After `BoundaryMesh::IO` exists, replace the old BoundaryLayer-to-IO link with:

```cmake
target_link_libraries(
    boundary_mesh_multi_normal
    PUBLIC BoundaryMesh::IO
)
```

- [ ] **Step 4: Move and retarget MultiNormal tests**

In `tests/CMakeLists.txt`, replace every moved source path with `unit/multi_normal/...` or `integration/multi_normal/...`. Link these targets to `BoundaryMesh::MultiNormal` instead of `BoundaryMesh::BoundaryLayer`:

```text
boundary_mesh_multi_normal_types_test
boundary_mesh_incident_face_fan_test
boundary_mesh_multi_normal_split_planner_test
boundary_mesh_blmesh_splitter_parity_test
boundary_mesh_blmesh_geometry_parity_test
boundary_mesh_multi_normal_topology_builder_test
boundary_mesh_multi_normal_quad_triangulator_test
boundary_mesh_multi_normal_transition_builder_test
boundary_mesh_multi_normal_intersection_resolver_test
boundary_mesh_blmesh_intersection_checker_test
boundary_mesh_multi_normal_transition_generator_test
boundary_mesh_multi_normal_growth_pipeline_test
boundary_mesh_multi_normal_mesh_merge_test
boundary_mesh_blmesh_topology_stitching_test
```

- [ ] **Step 5: Reconfigure and build the two module targets**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Release --target boundary_mesh_boundary_layer boundary_mesh_multi_normal
```

Expected: both static libraries build successfully. `boundary_mesh_boundary_layer` output must complete before any MultiNormal source is compiled; `boundary_mesh_multi_normal` then compiles the moved implementation.

- [ ] **Step 6: Verify the module boundary test is GREEN**

Run:

```powershell
ctest --test-dir build -C Release -R boundary_mesh_multi_normal_module_boundary_test --output-on-failure
```

Expected: `1/1` test passes.

Do not commit yet; Task 3 completes the required orchestration move so the full tree configures and links consistently.

---

### Task 3: Move combined orchestration into Transition

**Files:**
- Move: `include/boundary_mesh/growth/boundary_layer_generator.hpp` → `include/boundary_mesh/transition/boundary_layer_generator.hpp`
- Move: `src/growth/boundary_layer_generator.cpp` → `src/transition/boundary_layer_generator.cpp`
- Move: `tests/integration/boundary_layer_generation_pipeline_test.cpp` → `tests/integration/transition/boundary_layer_generation_pipeline_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Modify: all consumers of `boundary_mesh/growth/boundary_layer_generator.hpp`
- Modify: `include/boundary_mesh/transition/reserved_layer_transition.hpp`

**Interfaces:**
- Consumes: `generateMultiNormalTransition`, `generateRegularLayers`, and `mergeMultiNormalAndRegularMeshes` with their existing signatures.
- Produces: unchanged `generateBoundaryLayers(...)` API from `boundary_mesh/transition/boundary_layer_generator.hpp` and an upper-layer Transition target linking both modules.

- [ ] **Step 1: Move the generator and update its includes**

First extend `required_paths` in `tests/cmake/multi_normal_module_boundary_test.cmake` with:

```cmake
"include/boundary_mesh/transition/boundary_layer_generator.hpp"
"src/transition/boundary_layer_generator.cpp"
```

Extend `forbidden_paths` with:

```cmake
"include/boundary_mesh/growth/boundary_layer_generator.hpp"
"src/growth/boundary_layer_generator.cpp"
```

Run the focused boundary test and verify it fails with `Missing standalone multi-normal path: include/boundary_mesh/transition/boundary_layer_generator.hpp`.

Then use `git mv` for the header, implementation, and integration test. In the implementation, include:

```cpp
#include <boundary_mesh/transition/boundary_layer_generator.hpp>
```

In the moved header, use:

```cpp
#include <boundary_mesh/multi_normal/multi_normal_mesh_merge.hpp>
#include <boundary_mesh/multi_normal/multi_normal_transition_generator.hpp>
```

Keep all structures, errors, function parameters, and function bodies unchanged.

- [ ] **Step 2: Make Transition own the combined generator**

Add `src/transition/boundary_layer_generator.cpp` to `boundary_mesh_transition` and add the standalone module dependency:

```cmake
target_link_libraries(
    boundary_mesh_transition
    PUBLIC
        BoundaryMesh::MultiNormal
        BoundaryMesh::BoundaryLayer
        BoundaryMesh::Surface
        BoundaryMesh::Core
)
```

Remove any remaining MultiNormal include or link dependency from `boundary_mesh_boundary_layer`.

- [ ] **Step 3: Retarget the orchestration test**

Update the moved test source path to `integration/transition/boundary_layer_generation_pipeline_test.cpp`, include `boundary_mesh/transition/boundary_layer_generator.hpp`, and link its executable to:

```cmake
PRIVATE BoundaryMesh::Transition
```

- [ ] **Step 4: Update all remaining public include consumers**

Run:

```powershell
rg -n "boundary_mesh/growth/(multi_normal|blmesh|incident_face_fan|boundary_layer_generator)" include src tests
```

Expected: no matches. Update any match using the exact new `multi_normal/` or `transition/` path before continuing.

- [ ] **Step 5: Build and run focused tests**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Release --target boundary_mesh_transition boundary_mesh_boundary_layer_generation_pipeline_test
ctest --test-dir build -C Release -R "(multi_normal|boundary_layer_generation|reserved_layer_transition)" --output-on-failure
```

Expected: configuration succeeds and every selected test passes.

- [ ] **Step 6: Commit the completed module split**

Run:

```powershell
git add CMakeLists.txt tests/CMakeLists.txt tests/cmake include/boundary_mesh/multi_normal include/boundary_mesh/transition src/multi_normal src/transition tests/unit/multi_normal tests/integration/multi_normal tests/integration/transition
git add -u include/boundary_mesh/growth src/growth tests/unit/growth tests/integration
git commit -m "refactor: separate multi-normal module from growth"
```

Expected: one commit containing the structural test, moves, include rewrites, independent target, and orchestration relocation.

---

### Task 4: Verify module isolation and behavioral parity

**Files:**
- Verify only; no production file changes expected.

**Interfaces:**
- Consumes: completed `BoundaryMesh::BoundaryLayer`, `BoundaryMesh::MultiNormal`, and `BoundaryMesh::Transition` targets.
- Produces: build/test evidence that module separation preserved behavior.

- [ ] **Step 1: Verify forbidden paths and dependency references are absent**

Run:

```powershell
rg --files include/boundary_mesh/growth src/growth | Select-String 'multi_normal|blmesh|incident_face_fan|boundary_layer_generator'
rg -n "BoundaryMesh::MultiNormal|src/multi_normal|third/blmesh_mnormal" CMakeLists.txt
```

Expected: the first command returns no matches; the second shows ownership only in `boundary_mesh_multi_normal` and its upper-layer consumers.

- [ ] **Step 2: Build all Release targets**

Run with the normalized Windows environment used by this repository:

```powershell
python -c "import os,shutil,subprocess,sys; cmake=shutil.which('cmake'); env={k:v for k,v in os.environ.items() if k.lower()!='path'}; env['PATH']=os.environ.get('PATH') or os.environ.get('Path',''); sys.exit(subprocess.run([cmake,'--build','build','--config','Release','--target','ALL_BUILD'],env=env).returncode)"
```

Expected: exit code `0`.

- [ ] **Step 3: Run the complete test suite**

Run:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Expected: all original 72 tests plus `boundary_mesh_multi_normal_module_boundary_test` pass with zero failures.

- [ ] **Step 4: Check the final diff and worktree state**

Run:

```powershell
git diff --check HEAD~1 HEAD
git status --short --branch
git log -2 --oneline
```

Expected: no whitespace errors, a clean feature worktree, and the latest commit is `refactor: separate multi-normal module from growth`.
