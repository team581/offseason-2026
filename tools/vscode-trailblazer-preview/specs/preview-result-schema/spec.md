# Preview result data

The Java worker and `./gradlew previewAuto` use the same versioned JSON format. For the same source, auto, scope, robot, alliance, and seed, they return the same preview data.

- Include the schema version, auto class, scope, active segment ID, segment source ranges, robot ID, alliance, seed, and total duration in seconds. Reject unsupported schema versions.
- Include alliance-adjusted waypoints with position, positional tolerance, target heading, rotational tolerance, gate policy, marker, and 1-based Java source range where known.
- Include planned line and fillet geometry, sampled planned poses, and hard and margin keepout polygons. Apply the selected alliance transform to all field geometry and headings.
- Include wheel center-to-center dimensions from the selected robot's tuner constants, exterior bounds with 3-inch bumpers on each side, and the maximum forward intake extension for 2026. Report dimensions in meters.
- Include timestamped simulation samples with achieved pose, commanded and achieved speeds, path progress, heatmap speed, limiting factor, and the ratio of commanded speed to maximum speed.
- Include the closest achieved pass distance for each waypoint.
- Collect compile errors, path warnings, runtime diagnostics, heading tolerance misses, and keepout violations in one list. Include severity, code, and the relevant goal or timestamp when available.
