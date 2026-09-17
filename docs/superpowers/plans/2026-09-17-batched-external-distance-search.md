# Batched External Distance Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resolve all colliding external patches with shared global provisional builds.

**Architecture:** Track per-owner search intervals in a sorted map and advance all active owners once per global build/inspection round. Preserve the current exact collision checker and fallback rules.

**Tech Stack:** C++17, existing transition resolver and CTest.

## Global Constraints

- Keep single-owner search results unchanged.
- Do not change exact collision semantics.
- Preserve user-owned uncommitted files.

### Task 1: Multi-owner regression test

**Files:**
- Modify: `tests/unit/transition/layer_transition_resolver_test.cpp`

- [ ] Add two external owners with distinct safe thresholds.
- [ ] Count provisional builds and assert both owners converge while build count remains below 30.
- [ ] Run the test and verify it fails because the sequential implementation exceeds the bound.

### Task 2: Batched search

**Files:**
- Modify: `src/transition/layer_transition_resolver.cpp`

- [ ] Introduce per-owner low/high/probe/bracket state.
- [ ] Batch the halving phase into one build and inspect per round.
- [ ] Apply existing KeepHexa/dependent-high fallback for unbracketed owners.
- [ ] Batch all 12 binary-search rounds.
- [ ] Perform one final global build and retain existing final rollback inspection.
- [ ] Run focused and complete affected test groups.

### Task 3: Real-case verification

- [ ] Build Release CLI.
- [ ] Run anisotropic with 20 layers, first height `0.1`, growth ratio `1.2`.
- [ ] Report per-stage timings, cell counts, and any remaining bottleneck.
