# Single-high-edge pyramid ordering fix

## Problem

The single-high-edge quad transition constructs a VTK pyramid whose first four
vertices are not cyclic around the quadrilateral base. For example, the current
order `e,f,a,b,d` creates the crossing base edges `f-a` and `b-e`.

## Design

Fix the topology at its source in `appendSideTransition`; do not add a VTK-only
remapping or post-process generated files. Change the two diagonal branches to
emit cyclic base orders:

- `e,f,b,a,d`
- `f,e,a,b,c`

The fifth vertex remains the pyramid apex. Double-high-edge templates already
use cyclic orders and remain unchanged. Top-face triangles and tetrahedra are
also unchanged because their connectivity is independent of the ordering of
the pyramid's four base vertices.

## Testing

Update the single-high-edge transition regression test first and confirm that
it fails against the current implementation. Cover both diagonal choices so
both corrected orders are verified. Then make the minimal production change,
run the focused test, the full test suite, and regenerate the recovered 2dot5
case. Finally inspect cell 153585 and confirm its base is cyclic and the output
top surface still contains triangles only.

## Success criteria

- Every single-high-edge pyramid stores its first four vertices cyclically.
- Both diagonal branches are covered by regression tests.
- Existing transition counts and authored top faces do not change.
- The full test suite passes.
- The real 2dot5 run completes and its top VTK contains only type-5 cells.
