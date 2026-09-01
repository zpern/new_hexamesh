# Zero-layer faces skip corner lowering

## Goal

Prevent a source face that completes zero growth layers from directly lowering
faces around the non-contact corner of a selected high edge.

## Behavior

- A stopped face with `completed_layer == 0` does not select a high edge and
  does not apply `non_contact_corner_faces` constraints.
- Shared-edge layer-difference propagation remains enabled and continues to
  constrain neighboring faces through `TerminationPropagator::propagate()`.
- Stopped faces with `completed_layer > 0` retain the existing single-high-edge
  and non-contact-corner behavior.

## Implementation boundary

Add the zero-layer guard in
`TerminationPropagator::filterSingleHighEdgeCandidates()` before high-edge
selection. Do not change transition templates or the general propagation
algorithm.

## Verification

Add a unit regression case in which a face stops before its first layer while
an edge neighbor remains a candidate. Verify that a face incident only through
the selected edge's non-contact corner is not reduced to zero. Retain the
existing tests for positive-layer high-edge handling and layer-difference
propagation.
