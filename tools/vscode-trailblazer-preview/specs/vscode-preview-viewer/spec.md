# VS Code preview

Open the preview beside the Java editor when a Trailblazer auto opens or the preview command runs. Keep keyboard focus in the editor.

## Choose what to preview

- In Cursor mode, show the segment tied to the cursor's declaration, builder helper, or state branch. Show the whole auto when the cursor does not identify one segment.
- Show the complete executed segment, including waypoints added by helpers. In `RightTrenchAuto`, the second cycle includes its trench entry and shared finish.
- Whole Auto shows all segments in order and stays active until the user selects Cursor mode.
- Let users choose Blue or Red and robot 581, 9991, or 9981. Flip the path, waypoints, and keepouts for Red.
- Diff overlays the Git baseline trajectory and waypoints beneath the current ones. Show the signed difference in duration in seconds.

## Field and waypoints

- Use the high-resolution 2026 AdvantageScope field image with Blue-origin meter coordinates. Render at the display's pixel density and keep field landmarks aligned at every zoom.
- Draw the planned lines and fillets, the simulated trajectory as a speed heatmap, and the keepouts. Draw the selected robot's bumper and maximum forward intake footprint.
- Fit the displayed path with about 1 meter of padding per side. Scroll to zoom around the field point under the pointer.
- Pan and zoom smoothly when the displayed path changes. Draw a newly loaded path and robot footprint from start to finish in under 1 second.
- Use a bright orange dot and positional tolerance circle that stand out from field fuel. Use purple for the heading arrow and rotational tolerance wedge when available.
- Pulse the active waypoint. Highlight an editor location only when it maps to one executed waypoint in the active segment, including helper calls and multiline calls.
- Clicking a mapped waypoint reveals the same Java call and smoothly zooms in. Editing a mapped waypoint also zooms in, with enough space to see the path around the point.
- Start returning to the full displayed path 2 seconds after the click or latest edit. Keep that timing when a preview is pending or stale.

## Playback and status

- Place Play/Pause to the left of the timeline and play at 1× speed. Hover to scrub and click to pin a time.
- Leave at least 16 pixels between the timeline and each panel edge so users can scrub to either end without hitting the resize handle.
- Show duration, current time and speed, the active limiting factor and speed ratio, and any warnings or errors with their goal or timestamp.
- Keep the panel compact with control labels, marker names, readouts, and diagnostics. Show the pinned time without a separate badge.
- Refresh in place after edits settle or the file is saved. If compilation fails, keep the last valid path visible, mark it `STALE`, and show the compile error.
