# Track: Verify cmd_vel-Driven Motion-Mode Selection

**ID:** motion-mode-verify_20260728
**Type:** Feature
**Status:** Pending

## Documents

- [Specification](./spec.md)
- [Implementation Plan](./plan.md)

## Progress

- Phases: 0/4 complete
- Tasks: 0/15 complete

## Phases

1. [ ] Baseline and Instrumentation
2. [ ] Mode Reachability
3. [ ] Boundary Sweeps
4. [ ] Tuning and Finalization

## Notes

- Requires the **real chassis** for every phase after Phase 1. `ranger_can_sim` is
  deliberately not used — mode-switch timing is a hardware behavior.
- Depends on uncommitted changes in **two** repositories: `ranger_ros2`
  (`features/add_manager`) and `cmd_vel_manager` (`features/webot_added`).

## Quick Links

- [Back to Tracks](../../tracks.md)
- [Product Context](../../product.md)
- [Workflow](../../workflow.md)
