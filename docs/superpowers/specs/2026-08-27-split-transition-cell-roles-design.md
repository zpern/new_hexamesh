# Split Transition Cell Roles Design

## Goal

Remove the ambiguous `CellRole::Transition` value and represent multi-normal and reserved-layer transition cells as distinct roles.

## Design

`CellRole` contains exactly `RegularLayer`, `MultiNormalTransition`, and `ReservedLayerTransition`. Multi-normal construction assigns the multi-normal role; triangle and quad reserved-layer templates assign the reserved-layer role. The existing `reserved_transition_cell_count` counts only reserved-layer metadata before the multi-normal mesh merge.

The mesh-generation algorithms, cell ordering, and result structure remain unchanged. No compatibility value for the old role is retained, so missed call sites fail at compile time.

## Tests

Unit tests verify each producer emits its dedicated role and mesh merge preserves it. The combined integration test verifies multi-normal cells separately from reserved-layer cells and permits zero reserved-layer cells when the transformed front has no layer difference. Debug and Release full CTest suites must pass.
