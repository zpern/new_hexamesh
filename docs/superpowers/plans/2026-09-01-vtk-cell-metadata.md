# VTK Cell Metadata Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Write `source_face_id`, `layer`, and `cell_role` for every volume cell into legacy VTK output, then use them to diagnose cell 479558.

**Architecture:** Keep the surface writer unchanged. Extend the shared legacy writer with optional volume-cell metadata, require exact cell/metadata cardinality, and emit three independent `CELL_DATA` scalar arrays after `CELL_TYPES`.

**Tech Stack:** C++17, legacy ASCII VTK, CMake/MSBuild, existing executable unit tests.

## Global Constraints

- Metadata tuple order must exactly match `VolumeMesh::cells` order.
- Scalar names are exactly `source_face_id`, `layer`, and `cell_role`.
- Surface VTK output remains unchanged.
- Metadata/cell count mismatch fails without replacing the destination.
- Growth, coordination, transition templates, and multi-normal behavior do not change.

---

### Task 1: Persist volume-cell metadata

**Files:**
- Modify: `include/boundary_mesh/io/vtk_write_error.hpp`
- Modify: `src/io/legacy_vtk_writer.cpp`
- Test: `tests/unit/io/legacy_vtk_writer_test.cpp`
- Remove temporary diagnostics: `src/cli/boundary_mesh_command.cpp`

**Interfaces:**
- Consumes: `VolumeMesh::cells`, `VolumeMesh::metadata`, and `CellMetadata::{source_face_id,layer,role}`.
- Produces: existing `writeLegacyVtk(path, VolumeMesh)` with validated metadata output and `VtkWriteErrorCode::InvalidCellMetadataCount`.

- [ ] **Step 1: Write the failing metadata-output test**

Populate four records in cell order:

```cpp
volume.metadata = {
    {CellRole::RegularLayer, 11, 1},
    {CellRole::ReservedLayerTransition, 12, 2},
    {CellRole::MultiNormalTransition, 13, 3},
    {CellRole::RegularLayer, 14, 4}};
```

Assert the volume text contains `CELL_DATA 4` and the complete ordered values
under `SCALARS source_face_id unsigned_int 1`, `SCALARS layer unsigned_int 1`,
and `SCALARS cell_role int 1`. The expected role values are `0, 2, 1, 0`.
Also assert the surface text still contains no `CELL_DATA`.

- [ ] **Step 2: Run the writer test and verify RED**

Run:

```powershell
cmake --build .\build --config Release --target boundary_mesh_legacy_vtk_writer_test -j 1
.\build\Release\boundary_mesh_legacy_vtk_writer_test.exe
```

Expected: nonzero exit because the writer does not emit `CELL_DATA`.

- [ ] **Step 3: Add mismatch coverage and the dedicated error code**

Add `InvalidCellMetadataCount` to `VtkWriteErrorCode`. Construct an invalid
volume by removing one metadata record, pre-create its destination with sentinel
text, and assert the new error plus preservation of the sentinel file.

- [ ] **Step 4: Implement the minimal writer change**

Extend internal `writeCells()` with
`const std::vector<CellMetadata> *metadata`. Surface passes `nullptr`; volume
passes `&mesh.metadata`. Validate exact count before creating the temporary
file. After `CELL_TYPES`, emit:

```cpp
output << "CELL_DATA " << cells.size() << '\n';
output << "SCALARS source_face_id unsigned_int 1\n"
       << "LOOKUP_TABLE default\n";
for (const CellMetadata &value : *metadata)
    output << value.source_face_id << '\n';
output << "SCALARS layer unsigned_int 1\n"
       << "LOOKUP_TABLE default\n";
for (const CellMetadata &value : *metadata)
    output << value.layer << '\n';
output << "SCALARS cell_role int 1\n"
       << "LOOKUP_TABLE default\n";
for (const CellMetadata &value : *metadata)
    output << static_cast<int>(value.role) << '\n';
```

- [ ] **Step 5: Verify GREEN and regression tests**

Run the Step 2 commands and:

```powershell
ctest --test-dir .\build -C Release -R "boundary_mesh_legacy_vtk_writer_test|boundary_mesh_cgns_cli_pipeline_test" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 6: Remove the temporary CLI diagnostic and commit**

Delete only the `BOUNDARY_MESH_DIAGNOSTIC_CELL` block inserted after successful
growth. Preserve all pre-existing CLI changes. Commit the writer header,
implementation, and test as `feat: write volume cell metadata to VTK`.

### Task 2: Regenerate and inspect cell 479558

**Files:**
- Generate: `build/real_case/2dot5_cell_metadata/2dot5_cf_boundary_layer.vtk`
- Inspect: `src/growth/termination_propagator.cpp`
- Inspect: `src/transition/transition_layer_coordinator.cpp`
- Inspect: `src/transition/reserved_layer_transition.cpp`

**Interfaces:**
- Consumes: the three VTK `CELL_DATA` arrays from Task 1.
- Produces: an evidence-backed report mapping cell 479558 to its source face, layer, role, actual high edge, edge neighbors, and non-contact vertex one-ring.

- [ ] **Step 1: Rebuild the Release CLI**

Normalize duplicate Windows `PATH`/`Path` environment entries and run:

```powershell
cmake --build .\build --config Release --target boundary_mesh_cli -j 1
```

Expected: successful Release build.

- [ ] **Step 2: Regenerate the exact non-multi-normal case**

Run the user's input with first height `0.1`, ratio `1.2`, layer count `20`,
maximum skewness `1`, isotropic height `1.0`, no multi-normal flag, and output
prefix `build/real_case/2dot5_cell_metadata/2dot5_cf`. Do not pass trailing
positional arguments because this CLI rejects them.

- [ ] **Step 3: Read cell 479558 and its metadata**

Parse and report its VTK type, connectivity, `source_face_id`, `layer`, and
`cell_role`. Verify tuple 479558 in each scalar array aligns with connectivity
tuple 479558.

- [ ] **Step 4: Reconstruct the source-face neighborhood**

Using the original surface and `SurfaceTopology`, list ordered source vertices,
local edge neighbors, final trial/occupied layers, selected local high edge,
non-contact vertices, and all patch faces returned by `vertexFaces()` at those
vertices. Compare incident trial layers with the low face.

- [ ] **Step 5: Test one root-cause hypothesis**

Distinguish using evidence whether this is an internal Quad decomposition
pyramid, an older VTK with different cell ordering, a visually misidentified
high edge, or a real coordinator/corner-validation contradiction. Do not
propose a fix before one hypothesis is confirmed.
