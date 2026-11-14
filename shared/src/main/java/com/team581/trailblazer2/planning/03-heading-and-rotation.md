# Trailblazer 2 Design Specification: Holonomic Heading Scheduling & Coupling

**Document ID:** TB2-SPEC-03
**Status:** Implementation design
**Author:** Team 581 Technical Staff
**Applies To:** `HeadingPlanner`, `HeadingKeyframe`, `HeadingGate`

---

## 1. Overview

- **Trailblazer 1:** Heading was tied directly to each waypoint, switched targets abruptly halfway through curved paths (`t < 0.5`), couldn't specify clockwise vs. counter-clockwise turns, and used ad-hoc speed floors (`0.4 m/s`) or `360°` default tolerances that either stalled the robot or ignored rotation readiness.
- **Trailblazer 2:** Heading is a separate schedule $\theta^*(s)$ along the path:
  1. **Target Modes:** Face a fixed angle (`fixed`), face the direction of travel (`faceTravel`), continuously aim at a field point like the Hub (`facePoint`), or keep current heading (`hold`).
  2. **Explicit Turn Direction:** Choose `SHORTEST`, `CW`, or `CCW` so intakes or turrets always swing the right way.
  3. **Soft vs. Hard Gates:** A `SOFT` gate does not require heading readiness for passage. A `HARD` gate brakes to a hold if needed until measured heading readiness is satisfied. Both respect shared motion limits.
  4. **Shared Wheel Budget:** While spinning fast, `HeadingPlanner` automatically lowers the translation speed ceiling ($v_{max, eff}$) and friction budget ($a_{fric, eff}$) so outer swerve wheels don't saturate.

---

## 2. Decoupled Heading Schedule Architecture

In a holonomic swerve drive, translation and rotation are kinematically decoupled: the chassis can translate along an arbitrary path tangent $\hat{t}$ while independently rotating its azimuth $\theta$.

Trailblazer 2 models heading as a piecewise schedule $\theta^*(s)$ parameterized by path progress $s$.

```
Path Progress s:   0.0m                1.8m               3.2m             5.0m
Translation:       [Start] ---------- [Goal A] --------- [Goal B] ------- [Stop]
Heading Plan:      Face 0° ---------> Face 90° (Soft) -> Face Hub (Hard)-> Face 180°
```

### 2.1 Heading Keyframe Data Model

A heading schedule consists of an ordered sequence of `HeadingKeyframe` records:

```java
public record HeadingKeyframe(
    AnchorDistance activation,  // Start rotating here
    AnchorDistance anchor,      // Arrival: atGoal, before(m), after(m)
    HeadingTarget target,       // Target orientation rule: fixed, faceTravel, facePoint, hold
    TurnDirection direction,    // Rotation direction constraint: SHORTEST, CW, CCW
    GatePolicy gate,            // Impact on translational speed: SOFT vs HARD
    Rotation2d tolerance,       // Arrival angular tolerance (e.g. ±2.5 deg)
    double angularSpeedTolerance, // rad/s
    double dwellSeconds
) {}
```

#### Anchor Positions

- `atGoal(goalId)`: Keyframe aligns with goal arrival ($s = s_{goal}$).
- `beforeGoal(goalId, distanceMeters)`: Arrival is at $s = s_{goal} - d$.
- `afterGoal(goalId, distanceMeters)`: Arrival is at $s = s_{goal} + d$.
- `atProgress(sMeters)`: Fixed arc length along the stop-bounded leg.
- **Activation:** Defaults to the previous heading arrival or leg start. `beforeGoal` specifies when heading should be ready, not when rotation starts.
- **Validation:** Reject conflicting/out-of-order anchors. Resolve goal anchors using [01](01-path-geometry-and-kinematics.md), preserving keyframe identity across rebuilds.

#### Heading Target Modes

1. **Fixed Angle (`fixed(Rotation2d)`):** Target an absolute field heading (e.g. $0^\circ, 90^\circ, -180^\circ$).
2. **Face Travel (`faceTravel()`):** Continuously align the chassis with the velocity vector tangent:
   $$\theta^*(s) = \mathrm{atan2}\left(t_y(s), \; t_x(s)\right)$$
3. **Face Field Point (`facePoint(Supplier<Translation2d>)`):** Continuously aim a robot mechanism (turretless shooter or intake) at a fixed or dynamic field coordinate (e.g. Hub, Speaker, Reef, or human player station):
   $$\theta^*(s) = \mathrm{atan2}\left(y_{target} - y_{robot}(s), \; x_{target} - x_{robot}(s)\right)$$
4. **Hold Heading (`hold()`):** Capture measured heading once at keyframe entry.
5. **Undefined Direction:** At a zero-length path or coincident face-point target, retain the last valid target and report undefined aim readiness. Runtime resolves dynamic suppliers once per tick ([04](04-runtime-integration-and-teleop-assist.md)).

#### Turn Direction Disambiguation

- `SHORTEST`: Minimal absolute angular travel ($|\Delta \theta| \le \pi$).
- `CW`: Choose the clockwise target branch.
- `CCW`: Choose the counter-clockwise target branch.

_Why this matters in FRC:_ If the robot is at $0^\circ$ and needs to face $175^\circ$ while intaking balls from the right side, a `CCW` constraint ensures the intake sweeps toward the balls rather than spinning the back bumpers across the game pieces.

---

## 3. Angular Motion Profiling & Feedforward

To achieve smooth rotation without step changes, `HeadingPlanner` generates a continuous angular state $(\theta_{ref}, \omega_{ref})$ using an analytical trapezoidal profile bounded by measured drivetrain limits:

- $\omega_{max}$: Maximum angular velocity (rad/s)
- $\alpha_{max}$: Maximum angular acceleration (rad/s$^2$)

### 3.1 Unwrapped Angular Distance

At activation, seed reference angle/velocity from measured motion. Normalize the target difference once:
$$d=\mathrm{angleModulus}(\theta^*-\theta_{ref})\in[-\pi,\pi)$$

1. **`SHORTEST`:** $\Delta\theta=d$; choose a deterministic direction for the $\pi$ tie.
2. **`CW`:**
   $$\Delta\theta=\begin{cases}d&d\le0\\d-2\pi&d>0\end{cases}$$
3. **`CCW`:**
   $$\Delta\theta=\begin{cases}d&d\ge0\\d+2\pi&d<0\end{cases}$$
4. **Latch the branch:** $\theta_{goal}=\theta_{ref}+\Delta\theta$. Keep it unwrapped; do not request another full turn after overshoot. Moving targets unwrap against their previous target branch without resetting reference velocity.

Direction chooses the planned branch; bounded arrest and small overshoot corrections may require reversal.

### 3.2 Rotational Command Formulation & Reference State Evolution

1. **Advance a bounded profile:**
   $$(\theta_{ref,new},\omega_{ref,new})=\operatorname{TrapezoidStep}(\theta_{ref},\omega_{ref},\theta_{goal},0,\omega_{max},\alpha_{max},\Delta t)$$
   Handle initial wrong-direction motion and stopping distance. Preserve continuous angle and velocity; do not wrap stored reference angles or snap at an arbitrary error threshold.
2. **Add heading feedback:**
   $$\omega_{des}=\operatorname{clamp}\left(\omega_{ref,new}+kP_\theta\operatorname{angleModulus}(\theta_{ref,new}-\theta_{meas}),-\omega_{max},\omega_{max}\right)$$
3. **Bound the final command:**
   $$\omega_{cmd}=\omega_{prev}+\operatorname{clamp}(\omega_{des}-\omega_{prev},-\alpha_{max}\Delta t,\alpha_{max}\Delta t)$$
   Use previous applied angular velocity when available. For feasible inputs this respects speed and acceleration bounds; an already-out-of-bounds state enters bounded recovery ([02](02-velocity-profiler-and-limits.md)).
4. **Allocate from actual angular demand:**
   $$\alpha_{cmd}=\frac{\omega_{cmd}-\omega_{prev}}{\Delta t}$$
   Feed this final command/change into §5. Limiting reference acceleration alone does not limit feedback-induced acceleration.

- **State Invariant:** Reference state advances from its own previous state, not from measured heading each tick. Runtime owns that state; the profile step is pure math.

---

## 4. Translation-Rotation Coupling: Hard vs. Soft Gates

- **Soft Gate (`GatePolicy.SOFT`):** Heading readiness does not gate passage. Shared wheel/traction limits can still slow translation.
- **Hard Gate (`GatePolicy.HARD`):** Passage requires measured angle and angular speed within tolerance for the declared dwell. Mechanism readiness remains the caller's responsibility.

### 4.1 Hard Gate Policy: Braking, Holding, and Release

1. **Evaluate readiness:**
   $$e_\theta=\operatorname{angleModulus}(\theta^*-\theta_{meas})$$
   $$ready=\bigl(|e_\theta|\le\epsilon_\theta\bigr)\land\bigl(|\omega_{meas}|\le\omega_{tol}\bigr)$$
   Require `ready` continuously for the keyframe's dwell before opening the gate.
2. **Insert a barrier while closed:**
   $$v_{gate,k_{anchor}}=0$$
   The backward pass propagates braking upstream using available traction. Continue rotation while approaching/holding; do not set every upstream sample to zero.
3. **Hold at the anchor:** Reuse terminal 2D position convergence ([02 §5](02-velocity-profiler-and-limits.md#5-cross-track-control-stop-convergence--2d-vector-limiter)). A scalar zero-speed profile alone cannot correct residual position error.
4. **Release on readiness:** Remove the barrier and resume the profile from current motion state. Recheck readiness until passage. This temporary hold emits no stop-arrival marker and needs no caller release.
5. **Miss / stall:** If motion cannot stop before an unsatisfied gate or crosses it unready, report infeasibility/gate miss and fault with controlled braking. A configured watchdog faults a stalled gate; it never bypasses readiness or automatically backtracks.

### 4.2 Arrival-Time Estimates Are Not Gate Enforcement

The original pacing estimate assumed a particular approach-speed history:
$$v_{cap,kin}=\frac{2d_{anchor}}{t_{rot}}-v_{target}$$

- Changing traction, angular feedback, and dynamic targets invalidate that assumption.
- Keep any $t_{rot}$ estimate diagnostic; do not treat it as proof of measured arrival readiness.
- The initial implementation uses §4.1's explicit barrier and permits a pause. At zero distance/time, continue heading/position control without dividing by either.

---

## 5. Swerve Wheel Speed & Friction Allocation

Use the final $\omega_{cmd}$ and $\alpha_{cmd}$ from §3.2, module positions relative to the robot center, and field-to-robot conversion of translation.

1. **Exact module-speed check:** For module $i$ at $(x_i,y_i)$:
   $$\vec v_{module,i}=\begin{bmatrix}v_x-\omega_{cmd}y_i\\v_y+\omega_{cmd}x_i\end{bmatrix},\qquad\|\vec v_{module,i}\|\le v_{wheel,max}$$
2. **Conservative translational speed budget:**
   $$R_{max}=\max_i\sqrt{x_i^2+y_i^2},\qquad v_{max,eff}=\min\left(v_{max},\max(0,v_{wheel,max}-|\omega_{cmd}|R_{max})\right)$$
   Do not impose a positive translation floor when rotation consumes wheel capacity.
3. **Conservative translational acceleration budget:** Reserve angular tangential and centripetal demand:
   $$a_{rot}=|\alpha_{cmd}|R_{max}+\omega_{cmd}^2R_{max},\qquad a_{fric,eff}=\max(0,a_{fric}-a_{rot})$$
   Bound angular demand to the available envelope. This triangle-inequality model is conservative; it is not a tire-load or steering-rate model.
4. **Resolve competing demands:** Pass budgets to profiling and final vector allocation. Reduce angular change where possible if it would leave no feasible translational slew step; otherwise report bounded recovery ([02 §5](02-velocity-profiler-and-limits.md#5-cross-track-control-stop-convergence--2d-vector-limiter)).
5. **Verify applied output:** Check final combined module speeds after feedback/feedforward. Robot integration reports downstream saturation and owns steering/current constraints; kinematics alone does not enforce them.

Validate these modeled budgets with the independent plant/checks and carpet evidence in [06](06-verification-simulation-and-ci.md) and [07](07-empirical-calibration-routine.md).
