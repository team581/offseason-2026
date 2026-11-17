# Geometry and progress

- Implementation: `math/PathBuilder`, `model/FieldClearance`.
- A segment ends at a stop or a moving finish. Intermediate goals pass through.
- The path starts at measured position and joins goals with lines and circular fillets.
- A through goal's `withinMeters` bounds corner rounding.
- Adjacent fillets share available straight-line distance; keepouts can reduce a fillet.
- Exact reversals without a stop are rejected.
- Clearance checks use the rotated bumper rectangle and a margin around hard obstacles.
- A moving finish extends an overrun ray for handoff or braking.

## Progress

- Project measured position onto nearby path pieces.
- Restrict projection to the local goal window to avoid jumping across route crossings.
- Advance high-water progress only inside the tracking corridor and before a closed heading gate.
- Record each through marker when progress passes its anchor.
- Rebuild remaining geometry after an explicit relatch or sustained off-track error.
- Reset path-relative progress after rebuilding; retain already-passed markers.

## Examples

- Wider corner tolerance: more room for a fillet, subject to neighbors and clearance.
- Robot starts off the tape mark: build from its measured position.
- Robot is pushed off the route: relatch after the runtime's off-track dwell.
