# Trailblazer 2 Architecture & Design Specification Suite

**Document Suite ID:** TB2-SUITE-2026  
**System:** Team 581 Trailblazer 2 Autonomous & Teleoperated Motion Framework  
**Location:** `shared/src/main/java/com/team581/trailblazer2/planning/`  

---

## 1. Executive Overview

Trailblazer 2 is Team 581's pure-code, path-parameterized autonomous and teleoperated motion specification, planning, and following library. It succeeds Trailblazer 1 by introducing an online spatial planner with circular corner fillets, a two-pass dynamic programming velocity profiler, an isotropic 2D vector acceleration limiter, and decoupled holonomic heading schedules.

Trailblazer 2 is designed to be **robust by construction**, **physically feasible by construction**, **minimally tuned**, and **fully verifiable in simulation and CI**.

All auto specifications remain **100% in strongly-typed Java**, avoiding the "split-brain" failure modes of YAML/JSON, with a two-tier tooling strategy: an immediate MVP `./gradlew previewAuto` simulation tool with AdvantageScope export, followed by a standalone in-memory live previewer app (`TrailblazerStudio`) as the project's final deliverable.

---

## 2. Specification Document Index

The design specification suite is structured into nine dedicated technical documents:

| Spec ID | Document Title | Primary Scope & Architectural Responsibilities |
| :---: | :--- | :--- |
| **[TB2-SPEC-00](00-overview-and-architecture.md)** | **[System Architecture & Core Philosophy](00-overview-and-architecture.md)** | High-level system design, comparative analysis (Choreo, PathPlanner, 254), 6 core design principles, 5-stage runtime pipeline, minimal state model, and subsystem interfaces. |
| **[TB2-SPEC-01](01-path-geometry-and-kinematics.md)** | **[Path Geometry & Kinematics](01-path-geometry-and-kinematics.md)** | Polyline with circular fillets, deflection angle $\theta$, radius derivation from `within` tolerance, leg length tangent allocation, collinear/reversal edge cases, spatial projection $s$, and leg latching/re-latching. |
| **[TB2-SPEC-02](02-velocity-profiler-and-limits.md)** | **[Velocity Profiling & Physical Limits](02-velocity-profiler-and-limits.md)** | Drivetrain physical limits, 2-pass (backward/forward) dynamic programming spatial profiler, lateral centripetal limits, motor torque-speed curve, isotropic 2D vector slew rate limiter, and $< 35 \; \mu\text{s}$ zero-allocation execution. |
| **[TB2-SPEC-03](03-heading-and-rotation.md)** | **[Holonomic Heading Scheduling & Coupling](03-heading-and-rotation.md)** | Decoupled heading schedules $\theta^*(s)$, keyframe targets (`fixed`, `faceTravel`, `facePoint`, `hold`), rotational direction (`SHORTEST`, `CW`, `CCW`), trapezoidal angular feedforward, and hard/soft gate speed capping ($v_{cap} = d_{anchor} / t_{rot}$). |
| **[TB2-SPEC-04](04-teleop-and-dynamic-assist.md)** | **[Teleop Integration & Dynamic Assist](04-teleop-and-dynamic-assist.md)** | On-the-fly single-goal `driveTo`, dynamic vision target acquisition (AprilTags/game pieces), low-pass filtering, seamless driver stick preemption ($> 0.15$), continuous velocity handoff, and assisted semi-autonomous modes. |
| **[TB2-SPEC-05](05-spec-model-dsl-and-tooling.md)** | **[Java Specification Model, DSL & Tooling](05-spec-model-dsl-and-tooling.md)** | Canonical Java Fluent DSL (`Trailblazer.plan(...)`), centralized `FieldAnchors.java`, alliance mirroring, MVP `./gradlew previewAuto` task with AdvantageScope `.wpilog` & PNG exports, and the final In-Memory Live Previewer app (`TrailblazerStudio`). |
| **[TB2-SPEC-06](06-verification-simulation-and-ci.md)** | **[Verification, Simulation & CI](06-verification-simulation-and-ci.md)** | Injected deterministic `TimeSource` and `RobotStateSource`, 5-layer testing pyramid (unit, fuzzing, closed-loop auto sim, 20x Monte Carlo perturbation sweeps, log replay), golden metrics baselines, and automated PR comments. |
| **[TB2-SPEC-07](07-empirical-calibration-routine.md)** | **[Empirical Drivetrain Calibration](07-empirical-calibration-routine.md)** | Automated `CalibrateDrivetrainAuto` measuring $v_{max}$, $a_0$, $v_{free}$, $a_{brake}$, $a_{fric}$, $a_{lat}$, $\omega_{max}$, $\alpha_{max}$, and $\tau_{lag}$, generating `drivetrain-limits.json` to eliminate per-auto tuning. |
| **[TB2-SPEC-08](08-implementation-roadmap-and-migration.md)** | **[Implementation Roadmap & Migration](08-implementation-roadmap-and-migration.md)** | 5-phase incremental rollout with strict objective gates, backwards-compatibility adapter layer, two-tier tooling schedule, resolution of architectural open questions, and risk mitigation matrix. |
---

## 3. High-Level Pipeline Summary

Every 20 ms loop execution on the robot or in simulation:

```
[1. Measured State & Active Java Plan]
                │
                ▼
[2. PathBuilder: Build Horizon Polyline & Circular Fillets]
                │
                ▼
[3. ProgressTracker: Project Robot onto s, Update High-Water Mark & Latch]
                │
                ▼
[4. VelocityProfiler: 2-Pass Forward-Backward DP across Horizon]
                │
                ▼
[5. HeadingPlanner: Evaluate θ*(s), Compute t_rot, Apply Gate Speed Caps]
                │
                ▼
[6. VectorLimiter: Bound 2D Vector Delta ||Δv|| ≤ a_fric · dt]
                │
                ▼
[7. Output Field-Relative ChassisSpeeds + DogLog Diagnostics]
```

---

## 4. Quick Reference: Symptom to Physical Knob

When an auto behaves undesirably on the field, the author never tunes arbitrary PID gains. Adjustments are strictly tied to physical intent:

| Observable Field Symptom | What to Adjust | Where to Adjust | Document Reference |
| :--- | :--- | :--- | :--- |
| **Cuts corner too close to field obstacle** | Decrease `within` tolerance $\delta$ on that goal | Auto Code / `Goal.within()` | [TB2-SPEC-01 §2.3](01-path-geometry-and-kinematics.md#23-radius-bounds--derivation) |
| **Carries too little speed through corner** | Increase `within` tolerance $\delta$, or space neighboring goals farther apart | Auto Code / `Goal.within()` | [TB2-SPEC-01 §2.3](01-path-geometry-and-kinematics.md#23-radius-bounds--derivation) |
| **Robot hesitates / slows down waiting for rotation** | Move heading anchor earlier (e.g. `.before(1.0)`), or change gate to `SOFT` | Auto Code / `HeadingPlan` | [TB2-SPEC-03 §4](03-heading-and-rotation.md#4-translation-rotation-coupling-hard-vs-soft-gates) |
| **Rotates the long way around (intake exposed)** | Set `.direction(TurnDirection.CW)` or `.CCW` explicitly | Auto Code / `HeadingPlan` | [TB2-SPEC-03 §2.1](03-heading-and-rotation.md#21-heading-keyframe-data-model) |
| **Overshoots final stop goal on carpet** | Re-run `CalibrateDrivetrainAuto` to recalibrate $a_{brake}$ | Drivetrain Calibration | [TB2-SPEC-07 §3.1](07-empirical-calibration-routine.md#31-stage-1--2-straight-line-sprint--braking-test) |
| **Wobbles / oscillates across straight path line** | Verify cross-track time constant $\tau_{xt}$ and drive response lag $\tau_{lag}$ | Drivetrain Limits | [TB2-SPEC-02 §5](02-velocity-profiler-and-limits.md#5-cross-track-control-and-2d-vector-slew-limiter) |
| **Wheel slip / brownout during rapid direction reversal** | Verify friction limit $a_{fric}$ in `drivetrain-limits.json` | Drivetrain Limits | [TB2-SPEC-02 §5.2](02-velocity-profiler-and-limits.md#52-isotropic-2d-vector-acceleration-clamp) |
