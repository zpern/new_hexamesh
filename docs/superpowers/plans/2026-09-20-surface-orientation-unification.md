# Surface Orientation Unification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a transactional helper that makes every connected component of a `SurfaceMesh` locally winding-consistent.

**Architecture:** Build a temporary canonical-edge incidence map, then breadth-first propagate one flip bit per face. Validate the complete input and its orientation constraints before applying any recorded reversals, so an error never partially mutates the mesh.

**Tech Stack:** C++17, existing `Result<T, E>`, `std::variant`, CMake/CTest.

## Global Constraints

- Boundary edges are valid.
- The helper does not choose inward versus outward orientation.
- Success preserves vertices, face order, face types, and boundary tags.
- Failure leaves the complete `SurfaceMesh` unchanged.
- Processing is deterministic in ascending `SurfaceFaceId` order.

---

### Task 1: Specify the public contract and successful propagation

**Files:**
- Modify: `include/boundary_mesh/mesh/mesh_surface_orientation.hpp`
- Modify: `tests/unit/mesh/surface_orientation_test.cpp`
- Modify: `src/mesh/surface_orientation.cpp`

**Interfaces:**
- Consumes: `SurfaceMesh`, `Result<T, E>`, `DegenerateFace`, and `NonManifoldEdge`.
- Produces: `NonOrientableSurface`, `SurfaceOrientationError`, and `Result<std::size_t, SurfaceOrientationError> unifySurfaceOrientation(SurfaceMesh &mesh)`.

- [ ] **Step 1: Add failing success-path tests**

Add calls that require the new API to preserve an already consistent open pair, flip the second face of an inconsistent pair, independently process disconnected components, preserve tags and vertices, return the flip count, and return zero on a second call. Use triangles such as `{0,1,2}` and `{1,2,3}` for the inconsistent shared edge; after unification the second triangle must be `{1,3,2}`.

- [ ] **Step 2: Build to verify RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_surface_orientation_test
```

Expected: compilation fails because `unifySurfaceOrientation` is not declared.

- [ ] **Step 3: Declare the public result types and function**

Add to `mesh_surface_orientation.hpp`:

```cpp
struct NonOrientableSurface
{
    std::array<VertexId, 2> edge_vertices{};
    SurfaceFaceId first_face_id{};
    SurfaceFaceId second_face_id{};
};

using SurfaceOrientationError = std::variant<
    DegenerateFace,
    NonManifoldEdge,
    NonOrientableSurface>;

Result<std::size_t, SurfaceOrientationError>
unifySurfaceOrientation(SurfaceMesh &mesh);
```

Include the standard and project headers needed by those types.

- [ ] **Step 4: Implement minimal deterministic propagation**

In `surface_orientation.cpp`, add private `EdgeKey`, hashing, incidence, and face-reversal helpers. For every triangle or quad edge:

```cpp
const EdgeKey key{std::min(first, second), std::max(first, second)};
const bool canonical_direction = first < second;
```

Reject repeated vertices as `DegenerateFace`. Reject the third incidence on an edge as `NonManifoldEdge`. Propagate `flip[neighbor] = flip[current] ^ (current_direction == neighbor_direction)` with a FIFO queue. If an assigned neighbor disagrees with the required value, return `NonOrientableSurface`. Only after all components pass, reverse faces whose flip bit is set and return their count.

- [ ] **Step 5: Build and run to verify GREEN**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_surface_orientation_test
ctest --test-dir build -C Debug -R '^boundary_mesh_surface_orientation_test$' --output-on-failure
```

Expected: build succeeds and `1/1` test passes.

### Task 2: Specify errors and transactional behavior

**Files:**
- Modify: `tests/unit/mesh/surface_orientation_test.cpp`
- Modify: `src/mesh/surface_orientation.cpp` only if a failing test exposes missing behavior.

**Interfaces:**
- Consumes: `unifySurfaceOrientation`, `SurfaceOrientationError` from Task 1.
- Produces: verified error details and no-mutation guarantees.

- [ ] **Step 1: Add failing error-path tests**

Add three meshes and assert their exact alternatives and IDs:

```cpp
Triangle{{0, 0, 1}} // DegenerateFace{0}
```

Three triangles sharing `{0,1}` must return `NonManifoldEdge{{0,1},{0,1,2}}`. A closed parity-conflicting constraint graph must return `NonOrientableSurface` with its first deterministic conflicting edge and faces. Snapshot the complete mesh before each call and verify faces, vertices, and tags remain unchanged on failure.

- [ ] **Step 2: Run to verify RED**

Run the Debug build and targeted CTest command from Task 1. Expected: at least one new assertion fails if validation details or transactional behavior are incomplete.

- [ ] **Step 3: Make the smallest implementation correction**

Move all face mutation after incidence validation and BFS propagation. Populate error fields directly from the canonical edge and deterministic incidence order. Do not add global inward/outward classification.

- [ ] **Step 4: Run focused and topology regression tests**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_surface_orientation_test boundary_mesh_surface_topology_edge_error_test
ctest --test-dir build -C Debug -R 'boundary_mesh_surface_(orientation|topology_edge_error)_test' --output-on-failure
git diff --check
```

Expected: `2/2` tests pass and `git diff --check` reports no errors.

- [ ] **Step 5: Commit implementation**

```powershell
git add include/boundary_mesh/mesh/mesh_surface_orientation.hpp src/mesh/surface_orientation.cpp tests/unit/mesh/surface_orientation_test.cpp
git commit -m "feat: unify surface face orientation"
```

