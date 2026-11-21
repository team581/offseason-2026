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
- Advance high-water progress up to any closed heading gate.
- Record each through marker when progress passes its anchor.

## Examples

- Wider corner tolerance: more room for a fillet, subject to neighbors and clearance.
- Robot starts off the tape mark: build from its measured position when activating without a prepared path.
- Robot is pushed off the route: unified cross-track feedback steers back onto the fixed path.
