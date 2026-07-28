# Track: Light On/Off Topic

**ID:** light-control_20260728
**Type:** Feature
**Status:** Pending

## Documents

- [Specification](./spec.md)
- [Implementation Plan](./plan.md)

## Progress

- Phases: 0/3 complete
- Tasks: 0/11 complete

## Phases

1. [ ] Confirm Hardware Capability
2. [ ] Implement the Topic
3. [ ] Verify on the Robot

## Notes

- Adds `/light_control` (`std_msgs/Bool`) — `true` = both lights on, `false` = off.
- **Phase 1 is a go/no-go gate.** Not every Ranger variant has controllable lights; if
  this one doesn't, the track ends there with that recorded.
- Independent of `motion-mode-verify_20260728` — disjoint code, either order.

## Quick Links

- [Back to Tracks](../../tracks.md)
- [Product Context](../../product.md)
- [Workflow](../../workflow.md)
