# Trailblazer 2 design and implementation plan

TB2 is a Java motion library being prepared for the 2027 robot codebase. It combines spatial path following, circular fillets, constrained speed profiles, and independent heading control. Pure math is separated from runtime state and robot integration.

Start with [08: Implementation roadmap](08-implementation-roadmap-and-migration.md) for build order and [00: Architecture](00-overview-and-architecture.md) for boundaries and lifecycle. These documents describe intended implementation, not an already verified controller.

| Document | Owns |
| --- | --- |
| [00 — Architecture](00-overview-and-architecture.md) | Math/runtime separation, state, lifecycle, robot interfaces |
| [01 — Geometry](01-path-geometry-and-kinematics.md) | Fillets, anchors, projection, recovery |
| [02 — Velocity and limits](02-velocity-profiler-and-limits.md) | Sampling, braking/acceleration, final command constraints |
| [03 — Heading](03-heading-and-rotation.md) | Heading reference, directed turns, gates, shared wheel budget |
| [04 — Assist](04-teleop-and-dynamic-assist.md) | Driver ownership, handoff, dynamic targets |
| [05 — Authoring and tooling](05-spec-model-dsl-and-tooling.md) | Java plans, field context, shared runner, VS Code preview MVP |
| [06 — Verification](06-verification-simulation-and-ci.md) | Dedicated fixtures, independent checks, simulation, replay, CI |
| [07 — Calibration](07-empirical-calibration-routine.md) | Robot capabilities, parameter provenance, carpet evidence |
| [08 — Implementation roadmap](08-implementation-roadmap-and-migration.md) | Deliverables, dependency order, readiness criteria |

The current comp bot/new-member pathway provides intermittent testing opportunities. Old competition autos do not define compatibility or acceptance requirements. Plans and configuration stay in code; the VS Code preview and CI use the same Java execution path.
