# VTK Cell Metadata Design

## Goal

Persist the diagnostic metadata associated with every generated boundary-layer
volume cell in the legacy VTK output so that selecting a cell in ParaView shows
its source surface face, generated layer, and role.

## Output Format

The volume-mesh overload of `writeLegacyVtk()` appends a `CELL_DATA` section
after `CELL_TYPES`. It contains three independent scalar arrays whose tuple
order is exactly the volume-cell order:

- `source_face_id`: unsigned integer source surface-face identifier.
- `layer`: unsigned integer layer recorded by `CellMetadata`.
- `cell_role`: integer representation of `CellRole`.

Each scalar uses the standard legacy-VTK `SCALARS` plus `LOOKUP_TABLE default`
form. Independent arrays make all three values directly available for
ParaView coloring, filtering, and cell selection.

Surface-mesh VTK output remains unchanged because surface faces do not carry
`CellMetadata`.

## Validation and Errors

Before writing a volume mesh, the writer requires
`mesh.metadata.size() == mesh.cells.size()`. A mismatch returns a dedicated
write error and does not replace an existing destination file. This prevents
metadata from silently becoming misaligned with VTK cell IDs.

All existing vertex-reference, count-overflow, temporary-file, and atomic
replacement behavior remains unchanged.

## Testing

The writer unit test will first assert the desired metadata text against the
current writer and fail because no `CELL_DATA` section exists. The production
change will then make the test pass. Coverage includes:

- exact `CELL_DATA` tuple count and all three scalar names and values;
- metadata order matching cell order;
- rejection of a volume mesh whose metadata count differs from its cell count;
- unchanged surface VTK behavior;
- the existing writer and relevant CLI/integration tests.

After implementation, regenerate the `2dot5_cf` result and inspect VTK cell
479558 through the saved metadata to continue tracing its high-neighbor-edge
behavior.

## Scope

This change only persists metadata already present in `VolumeMesh`. It does not
change layer growth, transition coordination, cell templates, source-face
selection, or multi-normal behavior.
