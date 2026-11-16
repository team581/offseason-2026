# 03 — Heading and Rotation

**Applies To:** `HeadingPlanner`, `HeadingKeyframe`, `HeadingGate`

---

## 1. Overview

- **Trailblazer 1:** Waypoint-bound heading switched halfway through curves (`t < 0.5`) without turn-direction control. Speed floors (`0.4 m/s`) and `360°` default tolerances caused stalls or ignored readiness.

- **Trailblazer 3:** Heading is a separate schedule $\theta^*(s)$ along the path:
  1. **Target Modes:** Face a fixed angle (`fixed`), face the direction of travel (`faceTravel`), continuously aim at a scoring or field target (`facePoint`), or keep current heading (`hold`).
  2. **Explicit Turn Direction:** Choose `SHORTEST`, `CW`, or `CCW` to control rotation direction.
  3. **Soft vs. Hard Gates:** `SOFT` permits passage without heading readiness; `HARD` brakes to a hold until measured readiness. Both respect shared motion limits.
  4. **Shared Wheel Budget:** `RotationBudgetEstimator` reserves per-node rotational wheel-speed and traction budgets ($v_{max, eff, k}, a_{fric, eff, k}$) ahead of speed profiling, while `HeadingPlanner` bounds current-tick rotation without starving `SOFT` translation.

---

## 2. Decoupled Heading Schedule Architecture

Heading keyframes belong to the active named segment. Releasing an intermediate stop advances the horizon inside that segment. Activating another segment starts its authored schedule and seeds the heading reference from measured heading and applied angular velocity, with measured angular velocity as the fallback.

Swerve can translate along tangent $\hat{t}$ while independently rotating its azimuth $\theta$.

Trailblazer 3 models heading as a piecewise schedule $\theta^*(s)$ parameterized by path progress $s$.

```
Path Progress s:   0.0m                1.8m               3.2m             5.0m
Translation:       [Start] ---------- [Goal A] --------- [Goal B] ------- [Stop]
Heading Plan:      Face 0° ---------> Face 90° (Soft) -> Aim Target (Hard)-> Face 180°
```

### 2.1 Heading Keyframe Data Model

Use ordered `HeadingKeyframe` records:

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

- **Validation & Relatch Clamping:** Reject conflicting/out-of-order anchors on the nominal authored horizon. Resolve goal anchors using [01](01-path-geometry-and-kinematics.md), preserving keyframe identity across rebuilds; during dynamic rebuilds, relatches, or `activate(plan, startAtGoalId)`, clamp relative `beforeGoal` / `afterGoal` positions to $[0, S_{total}]$ (while preserving keyframe order) so recovery closer than `beforeGoal`'s offset never faults. Keep static vs. live-field `facePoint` targets type-distinct ([05 §3](05-developer-tooling.md)): `facePoint(Translation2d)` transforms with alliance symmetry, whereas `facePointDynamic(...)` consumes Blue-origin field coordinates untouched (and any robot-frame mechanism angle offset $\psi_{mech}$ is hardware geometry and is never negated under `MIRROR_X`).

#### Heading Target Modes

1. **Fixed Angle (`Goal.faceDegrees(...)` or `Goal.face(Rotation2d)`):** Target an absolute field heading (0° along field +X, positive angles counterclockwise), transformed for the active alliance. The target changes at activation with $\theta^{*\prime}(s) = 0$; the chassis command ramps under §3.2. Soft targets do not hold translation for heading readiness. Use segment angular speed and acceleration ceilings to pace turns that should be gentler than the robot maximum.
2. **Face Travel (`faceTravel()`):** Continuously align the chassis with the velocity vector tangent, with spatial derivative $\theta^{*\prime}(s) = 0$ on straight lines and $\theta^{*\prime}(s) = \sigma_{turn,k}/r_k$ on fillet arcs:

   ```math
   \theta^*(s) = \mathrm{atan2}\left(t_y(s), \; t_x(s)\right)
   ```

3. **Face Field Point (`facePoint(Supplier<Translation2d>)`):** Continuously aim a mechanism at a fixed or dynamic field coordinate with $\vec{\Delta}(s) = \mathbf{p}_{target} - \mathbf{r}(s)$:

   ```math
   \theta^*(s) = \mathrm{atan2}\left(\Delta_y(s), \; \Delta_x(s)\right), \qquad
   \theta^{*\prime}(s) = w(\|\vec{\Delta}\|)\,\frac{-\Delta_x t_y + \Delta_y t_x}{\Delta_x^2 + \Delta_y^2}
   ```

   where $w(\|\vec{\Delta}\|) = \mathrm{clamp}((\|\vec{\Delta}\| - 0.25) / 0.15,\; 0,\; 1)$ ramps derivative smoothly to zero. When $\|\vec{\Delta}\| < r_{min\_aim}$ (`0.25 m`), latch/freeze $\theta^*$ at its last valid angle, set $\theta^{*\prime}(s) = 0$, and report `AimReadiness.SINGULAR_HOLD` (preventing $180^\circ$ flips on close passes).
4. **Hold Heading (`hold()`):** Capture measured heading once at keyframe entry with $\theta^{*\prime}(s) = 0$.
5. **Undefined Direction:** At a zero-length path or coincident face-point target, retain the last valid target and report `AimReadiness.SINGULAR_HOLD`. Runtime resolves dynamic suppliers once per tick ([04](04-runtime-integration-and-teleop-assist.md)).

#### Turn Direction Disambiguation

- `SHORTEST`: Minimal absolute angular travel ($|\Delta \theta| \le \pi$).
- `CW`: Choose the clockwise target branch.
- `CCW`: Choose the counter-clockwise target branch.

Example: rotating from $0^\circ$ to $175^\circ$ with `CCW` sweeps the intake toward balls on the right.

---

## 3. Angular Motion Profiling & Feedforward

`HeadingPlanner` generates continuous angular state $(\theta_{ref}, \omega_{ref})$ using an analytical trapezoidal profile bounded by measured drivetrain limits:

- $\omega_{max}$: Maximum angular velocity (rad/s)
- $\alpha_{max}$: Maximum angular acceleration (rad/s$^2$)

### 3.1 Unwrapped Angular Distance

At keyframe activation, seed reference angle/velocity from prior reference state (or measured motion on activation) and normalize the target difference **once** for that keyframe ID:

```math
d=\mathrm{angleModulus}(\theta^*_{raw,0}-\theta_{ref})\in[-\pi,\pi)
```

1. **`SHORTEST`:** $\Delta\theta=d$; choose a deterministic direction for the $\pi$ tie.
2. **`CW`:**

   ```math
   \Delta\theta=\begin{cases}d&d\le0\\d-2\pi&d>0\end{cases}
   ```

3. **`CCW`:**

   ```math
   \Delta\theta=\begin{cases}d&d\ge0\\d+2\pi&d<0\end{cases}
   ```

4. **One-shot branch latch & continuous unwrapping:** Seed $\theta^*_{unwrapped,0}=\theta_{ref}+\Delta\theta$. On subsequent ticks of the same keyframe ID, never re-evaluate `CW`/`CCW` branching; evolve strictly by continuity so target noise cannot trigger a $360^\circ$ wrap:

   ```math
   \theta^*_{unwrapped,n} = \theta^*_{unwrapped,n-1} + \mathrm{angleModulus}\left(\theta^*_{raw,n} - \theta^*_{unwrapped,n-1}\right)
   ```

Direction chooses the planned branch; bounded arrest and small overshoot corrections may require reversal.

### 3.2 Rotational Command Formulation & Reference State Evolution

1. **Advance bounded profile in the moving-target error frame:**
   Evaluate $\theta^*_{unwrapped}$ and target angular feedforward $\omega^* = \mathrm{clamp}(\theta^{*\prime}(s_{eval,t}) v_{profile}, -\omega_{eff}, \omega_{eff})$ using the active budgets $(\omega_{eff}, \alpha_{eff})$ from §5. Form the relative error state:

   ```math
   e_0 = \theta_{ref} - \theta^*_{unwrapped}, \qquad \dot{e}_0 = \omega_{ref} - \omega^*
   ```

   with directional relative speed bound $\dot{e}_{max} = \max(0, \omega_{eff} - \omega^*)$ when $e_0 < 0$ (or $e_0 = 0, \dot{e}_0 > 0$), else $\max(0, \omega_{eff} + \omega^*)$. When $\alpha_{eff} \le 10^{-6}$, hold $\omega_{ref,new} = \omega_{ref}$; when $\dot{e}_{max} \le 10^{-4}\text{ rad/s}$ (feedforward saturation or zero speed budget), bypass `TrapezoidStep` (whose cruise-distance division would yield `NaN`) and apply bounded velocity arrest $\dot{e}_{new} = \dot{e}_0 + \mathrm{clamp}(-\dot{e}_0, -\alpha_{eff}\Delta t, \alpha_{eff}\Delta t)$. Otherwise step `TrapezoidStep` toward $(0, 0)$ in the error frame to avoid point-to-point braking chatter when $\omega^* \ne 0$:

   ```math
   (e_{new}, \dot{e}_{new}) = \mathrm{TrapezoidStep}(e_0, \dot{e}_0, 0, 0, \dot{e}_{max}, \alpha_{eff}, \Delta t)
   ```

   ```math
   \omega_{ref,new} = \mathrm{clamp}(\omega^* + \dot{e}_{new}, -\omega_{eff}, \omega_{eff}), \qquad
   \theta_{ref,new} = \theta_{ref} + \frac{1}{2}(\omega_{ref} + \omega_{ref,new})\Delta t, \qquad
   \alpha_{ref} = \frac{\omega_{ref,new} - \omega_{ref}}{\Delta t}
   ```

   Preserve continuous angle and velocity; do not wrap stored reference angles or snap at an arbitrary error threshold.
2. **Add heading feedback:**

   ```math
   \omega_{des}=\mathrm{clamp}\left(\omega_{ref,new}+kP_\theta\mathrm{angleModulus}(\theta_{ref,new}-\theta_{meas}),-\omega_{eff},\omega_{eff}\right)
   ```

3. **Bound the final command:**

   ```math
   \omega_{cmd}=\omega_{prev}+\mathrm{clamp}(\omega_{des}-\omega_{prev},-\alpha_{max}\Delta t,\alpha_{max}\Delta t)
   ```

   Use TB3's previous `lastCommand` angular velocity during continuous tracking (measured/applied on activation or disturbance reseed). Out-of-bounds states enter bounded recovery ([02](02-velocity-profiler-and-limits.md)).
4. **Allocate from final angular demand:** Pass $\omega_{cmd}$ directly to the 4 exact module-speed constraints in [02 §5](02-velocity-profiler-and-limits.md#5-cross-track-control-stop-convergence-7-set-2d-projection--feedforwards), and pass noise-free $\alpha_{ref}$ (never differentiated gyro feedback) as the angular acceleration feedforward.

- **State Invariant:** Advance from previous reference state, not measured heading each tick. Runtime owns state; the profile step is pure math.

---

## 4. Translation-Rotation Coupling: Hard vs. Soft Gates

- **Soft Gate (`GatePolicy.SOFT`):** Heading readiness does not gate passage. Shared wheel/traction limits can still slow translation.

- **Hard Gate (`GatePolicy.HARD`):** Passage requires measured angle and angular speed within tolerance for the declared dwell. Mechanism readiness remains the caller's responsibility.

### 4.1 Hard Gate Policy: Braking, Holding, and Release

1. **Evaluate readiness:**

   ```math
   e_\theta=\mathrm{angleModulus}(\theta^*-\theta_{meas})
   ```

   ```math
   ready=\bigl(|e_\theta|\le\epsilon_\theta\bigr)\land\bigl(|\omega_{meas}|\le\omega_{tol}\bigr)\land\bigl(|\theta^*_{unwrapped}-\theta_{ref}|<\tfrac{\pi}{2}\bigr)
   ```

   (Requiring $|\theta^*_{unwrapped} - \theta_{ref}| < \pi/2$ prevents near-$360^\circ$ directed `CW`/`CCW` turns from latching `ready` before starting their sweep.) Require `ready` continuously for `dwellSeconds` inside `maxGateHoldRadius` before opening the gate.
2. **Insert a barrier while closed:**

   ```math
   v_{gate,k_{anchor}}=0
   ```

   Clamp $s_{high\_water} \le s_{anchor}$ (paired with [01 §6](01-path-geometry-and-kinematics.md#goal-anchors-and-passage)'s closed-gate guard) while the gate is closed so overshoot cannot record downstream markers as passed. The backward pass propagates braking upstream using available traction. Continue rotation while approaching/holding; do not set every upstream sample to zero.
3. **Hold & pullback at the anchor:** Treat an unsatisfied `HARD` gate as a 2D spatial hold point $\mathbf{p}_{gate} = \mathbf{r}(s_{anchor})$ using [02 §5](02-velocity-profiler-and-limits.md#5-cross-track-control-stop-convergence-7-set-2d-projection--feedforwards) terminal convergence. Braking overshoot within `maxGateHoldRadius` (default `0.25 m`) pulls back to $\mathbf{p}_{gate}$ while finishing rotation rather than faulting.
4. **Latch open on readiness:** Once `ready` holds for `dwellSeconds` within `maxGateHoldRadius`, **latch the gate open** with $2\epsilon_\theta$ dropout hysteresis until anchor passage, removing the zero-speed barrier so resume transients do not slam it shut again. Emit no stop-arrival marker and require no caller release.
5. **Miss / stall:** If overshoot exceeds `maxGateHoldRadius`, motion cannot recover within `maxGateHoldRadius`, or the gate watchdog expires while closed, report infeasibility/gate miss and fault with controlled braking; never bypass readiness.

### 4.2 Arrival-Time Estimates Are Not Gate Enforcement

The original pacing estimate assumed a particular approach-speed history:

```math
v_{cap,kin}=\frac{2d_{anchor}}{t_{rot}}-v_{target}
```

- Changing traction, angular feedback, and dynamic targets invalidate that assumption.
- Keep any $t_{rot}$ estimate diagnostic; do not treat it as proof of measured arrival readiness.
- The initial implementation uses §4.1's explicit barrier and permits a pause. At zero distance/time, continue heading/position control without dividing by either.

---

## 5. Swerve Wheel Speed, Friction & Spatial Budget Allocation

Allocate shared wheel speed and tire-carpet traction in two places: **per-node budgets ahead of `VelocityProfiler`** (using noise-free reference schedules) and **exact module-speed projection in `VectorLimiter`** (using final $\omega_{cmd}$).

1. **Exact module-speed constraint:** For module $i$ at robot-frame $(x_i,y_i)$ with $v_{wheel,max} = v_{max}$:

   ```math
   \vec v_{module,i}=\begin{bmatrix}v_x-\omega_{cmd}y_i\\v_y+\omega_{cmd}x_i\end{bmatrix},\qquad\|\vec v_{module,i}\|\le v_{max}
   ```

2. **Conservative translational speed budget:**

   ```math
   R_{max}=\max_i\sqrt{x_i^2+y_i^2},\qquad v_{max,eff}=\min\left(v_{max},\max(0,v_{max}-|\omega|R_{max})\right)
   ```

   Do not impose a positive translation floor when rotation consumes wheel capacity.
3. **Rigid-body rotational friction reserve:** During pure spin around the CoM ($\alpha = 0$), internal chassis frame tension—not ground tire friction—carries $-\omega^2 \vec{r}_i$. Only yaw inertia $\gamma_J = \min\left(1.0,\; \frac{4 J_{zz}}{m \sum_{j=0}^{3} (x_j^2 + y_j^2)}\right)$ and parasitic steering/CG-offset scrub ($c_{parasitic} = 0.12$) consume ground friction in $L_2$:

   ```math
   a_{rot}(\omega, \alpha) = R_{max}\sqrt{\left(\gamma_J |\alpha|\right)^2 + \left(c_{parasitic}\omega^2\right)^2}, \qquad
   a_{fric,eff} = \max\left(0,\; a_{fric} - a_{rot}(\omega, \alpha)\right)
   ```

4. **Speed-blended rotation budget share:** Protect mid-field `SOFT` translation from wheel or traction starvation while allowing full rotation capacity near stops, closed `HARD` gates, and stationary `snapAngle`:

   ```math
   \rho(v) = \rho_{move} + (1 - \rho_{move})\,\mathrm{clamp}\left(1 - \frac{v}{0.50\text{ m/s}},\; 0,\; 1\right), \qquad
   \rho_{\omega,move} = 0.45,\; \rho_{a,move} = 0.40
   ```

   ```math
   \omega_{eff}(v) = \min\left(\omega_{max},\; \rho_\omega(v)\frac{v_{max}}{R_{max}},\; \sqrt{\frac{0.8\,\rho_a(v)\,a_{fric}}{c_{parasitic} R_{max}}}\right), \qquad
   \alpha_{eff}(v) = \min\left(\alpha_{max},\; \frac{\rho_a(v)\,a_{fric}}{\gamma_J R_{max}}\right)
   ```

   (This guarantees angular braking retains at least $60\%$ of the rotational traction share at peak spin.)
5. **Per-node spatial horizon budgets (`RotationBudgetEstimator`):** Before `VelocityProfiler` runs, compute noise-free $(v_{max,eff,k}, a_{fric,eff,k})$ at every node $k$ from:
   - The closed-form path-coupled wheel limit $v_{coupled,max,k} = \frac{v_{max}}{1 + |\theta^{*\prime}|_k R_{max}}$ on `faceTravel`/`facePoint` segments;
   - Scheduled turn footprints downstream of $s_{act,m}$: for each turn $|\Delta\theta_m|$ using upper-bound cruise speed $v_{est} = \min(v_{max}, v_{zone,k}, v_{corner,nom,k})$, reserve rotational budgets over $\Delta s_{rot,m} = v_{est}\left(\frac{|\Delta\theta_m|}{\omega_{eff}(v_{est})} + \frac{\omega_{eff}(v_{est})}{\alpha_{eff}(v_{est})}\right)$ (including acquisition jump $\Delta\theta_{acq}$ when entering `faceTravel`/`facePoint`); and
   - Local stopping-distance decay of the current reference state $(\omega_{ref}, \alpha_{ref})$ near $s_{meas}$.
   Never feed differentiated gyro feedback $\alpha_{cmd} = (\omega_{cmd} - \omega_{prev})/\Delta t$ into the spatial horizon; apply exact $\omega_{cmd}$ only in the 7-constraint final projection ([02 §5](02-velocity-profiler-and-limits.md#5-cross-track-control-stop-convergence-7-set-2d-projection--feedforwards)).
6. **Verify applied output:** Check final combined module speeds after feedback/feedforward. Robot integration reports downstream saturation and owns steering/current constraints; kinematics alone does not enforce them.

Validate these modeled budgets with the independent plant/checks and carpet evidence in [06](06-verification-simulation-and-ci.md) and [07](07-empirical-calibration-routine.md).
