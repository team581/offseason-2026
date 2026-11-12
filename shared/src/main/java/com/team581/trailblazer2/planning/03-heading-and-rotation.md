# Trailblazer 2 Design Specification: Holonomic Heading Scheduling & Coupling

**Document ID:** TB2-SPEC-03
**Status:** Approved for Implementation
**Author:** Team 581 Technical Staff
**Applies To:** `HeadingPlanner`, `HeadingKeyframe`, `HeadingGate`

---

## 1. Overview

- **Trailblazer 1:** Heading was tied directly to each waypoint, switched targets abruptly halfway through curved paths (`t < 0.5`), couldn't specify clockwise vs. counter-clockwise turns, and used ad-hoc speed floors (`0.4 m/s`) or `360°` default tolerances that either stalled the robot or ignored rotation readiness.
- **Trailblazer 2:** Heading is a separate schedule $\theta^*(s)$ along the path:
  1. **Target Modes:** Face a fixed angle (`fixed`), face the direction of travel (`faceTravel`), continuously aim at a field point like the Hub (`facePoint`), or keep current heading (`hold`).
  2. **Explicit Turn Direction:** Choose `SHORTEST`, `CW`, or `CCW` so intakes or turrets always swing the right way.
  3. **Soft vs. Hard Gates:** A `SOFT` gate rotates as fast as possible without slowing down driving. A `HARD` gate calculates how many seconds the turn will take ($t_{rot}$) and paces driving speed so the robot arrives already facing the target angle.
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
    AnchorDistance anchor,      // Spatial location along the path: atGoal, before(m), after(m)
    HeadingTarget target,       // Target orientation rule: fixed, faceTravel, facePoint, hold
    TurnDirection direction,    // Rotation direction constraint: SHORTEST, CW, CCW
    GatePolicy gate,            // Impact on translational speed: SOFT vs HARD
    Rotation2d tolerance        // Arrival angular tolerance (e.g. ±2.5 deg)
) {}
```

#### Anchor Positions

- `atGoal(goalId)`: Keyframe aligns with goal arrival ($s = s_{goal}$).
- `beforeGoal(goalId, distanceMeters)`: Keyframe triggers at $s = s_{goal} - d$.
- `afterGoal(goalId, distanceMeters)`: Keyframe triggers at $s = s_{goal} + d$.
- `atProgress(sMeters)`: Fixed arc length along the segment.

#### Heading Target Modes

1. **Fixed Angle (`fixed(Rotation2d)`):** Target an absolute field heading (e.g. $0^\circ, 90^\circ, -180^\circ$).
2. **Face Travel (`faceTravel()`):** Continuously align the chassis with the velocity vector tangent:
   $$\theta^*(s) = \mathrm{atan2}\left(t_y(s), \; t_x(s)\right)$$
3. **Face Field Point (`facePoint(Supplier<Translation2d>)`):** Continuously aim a robot mechanism (turretless shooter or intake) at a fixed or dynamic field coordinate (e.g. Hub, Speaker, Reef, or human player station):
   $$\theta^*(s) = \mathrm{atan2}\left(y_{target} - y_{robot}(s), \; x_{target} - x_{robot}(s)\right)$$
4. **Hold Heading (`hold()`):** Maintain the current heading at keyframe entry.

#### Turn Direction Disambiguation

- `SHORTEST`: Minimal absolute angular travel ($|\Delta \theta| \le \pi$).
- `CW`: Force clockwise rotation ($\dot{\theta} \le 0$).
- `CCW`: Force counter-clockwise rotation ($\dot{\theta} \ge 0$).

_Why this matters in FRC:_ If the robot is at $0^\circ$ and needs to face $175^\circ$ while intaking balls from the right side, a `CCW` constraint ensures the intake sweeps toward the balls rather than spinning the back bumpers across the game pieces.

---

## 3. Angular Motion Profiling & Feedforward

To achieve smooth rotation without step changes, `HeadingPlanner` generates a continuous angular state $(\theta_{ref}, \omega_{ref})$ using an analytical trapezoidal profile bounded by measured drivetrain limits:

- $\omega_{max}$: Maximum angular velocity (rad/s)
- $\alpha_{max}$: Maximum angular acceleration (rad/s$^2$)

### 3.1 Unwrapped Angular Distance

Given current measured heading $\theta$ and target $\theta^*$:

1. If `SHORTEST`:
   $$\Delta \theta = \mathrm{angleModulus}(\theta^* - \theta) \in [-\pi, \pi]$$
2. If `CW`:
   $$\Delta \theta = \begin{cases} \theta^* - \theta & \text{if } \theta^* - \theta \le 0 \\ (\theta^* - \theta) - 2\pi & \text{if } \theta^* - \theta > 0 \end{cases}$$
3. If `CCW`:
   $$\Delta \theta = \begin{cases} \theta^* - \theta & \text{if } \theta^* - \theta \ge 0 \\ (\theta^* - \theta) + 2\pi & \text{if } \theta^* - \theta < 0 \end{cases}$$

### 3.2 Rotational Command Formulation & Reference State Evolution

Every loop, the rotational profiler advances its internal reference state $(\theta_{ref}, \omega_{ref})$ and computes the rotational command:
$$\omega_{stop\_lim} = \mathrm{sgn}(\Delta \theta) \sqrt{2 \alpha_{max} |\Delta \theta|}, \quad \omega_{target} = \mathrm{clamp}\left(\omega_{stop\_lim}, \; -\omega_{max}, \; \omega_{max}\right)$$
$$\omega_{ref}(t) = \omega_{ref}(t - \Delta t) + \mathrm{clamp}\left(\omega_{target} - \omega_{ref}(t - \Delta t), \; -\alpha_{max} \Delta t, \; \alpha_{max} \Delta t\right)$$
$$\theta_{ref}(t) = \begin{cases} \theta^* & \text{if } |\Delta \theta| < 0.15\text{ rad} \\ \mathrm{angleModulus}\left(\theta_{ref}(t - \Delta t) + \omega_{ref}(t) \Delta t\right) & \text{otherwise} \end{cases}$$
$$\omega_{cmd} = \mathrm{clamp}\left(\omega_{ref}(t) + kP_\theta \cdot \mathrm{angleModulus}(\theta_{ref}(t) - \theta), \; -\omega_{max}, \; \omega_{max}\right)$$

- **State Invariant:** $\theta_{ref}(t)$ evolves from its own previous reference $\theta_{ref}(t - \Delta t) + \omega_{ref}(t) \Delta t$ (snapping to $\theta^*$ inside $0.15\text{ rad}$), **never** from $\theta + \omega_{ref} \Delta t$ (which would collapse $\theta_{ref} - \theta$ into $\omega_{ref} \Delta t$ and disable closed-loop disturbance rejection).

---

## 4. Translation-Rotation Coupling: Hard vs. Soft Gates

- **Soft Gate (`GatePolicy.SOFT`):** Rotate as fast as $\omega_{max}, \alpha_{max}$ allow; never slow down translation. Best for field transit.
- **Hard Gate (`GatePolicy.HARD`):** Robot must arrive at $s_{anchor}$ with $|\theta - \theta^*| \le \epsilon_\theta$. Best for scoring shots, intaking, and docking.

### 4.1 Hard Gate Policy: Remaining Rotation Time $t_{rot}$ & Velocity Ceiling $v_{cap}$

1. **Calculate Remaining Rotation Time $t_{rot}$:**
   Given unwrapped error $\Delta \theta$ (§3.1) and arrival tolerance $\epsilon_\theta$, net angular travel is $|\Delta \theta_{net}| = \max(0.0, |\Delta \theta| - \epsilon_\theta)$.
   - **Already Within Angle Tolerance ($|\Delta \theta_{net}| = 0$):**
     $$t_{rot} = \begin{cases} 0.0 & \text{if } |\omega_{meas}| \le \omega_{tol} \;\;(0.35\text{ rad/s}) \\ \dfrac{|\omega_{meas}|}{\alpha_{max}} & \text{if } |\omega_{meas}| > \omega_{tol} \;\;(\text{spin-arrest time}) \end{cases}$$
   - **Rotating Opposite to Target ($\omega_{meas} \cdot \Delta \theta < 0$):**
     Stop current spin ($t_{turn} = |\omega_{meas}|/\alpha_{max}$, $d_{turn} = \omega_{meas}^2 / (2 \alpha_{max})$), then traverse $|\Delta \theta|_{eff} = |\Delta \theta_{net}| + d_{turn}$ from rest:
     $$t_{rot} = t_{turn} + \begin{cases} 2 \sqrt{\dfrac{|\Delta \theta|_{eff}}{\alpha_{max}}} & \text{if } |\Delta \theta|_{eff} \le \dfrac{\omega_{max}^2}{\alpha_{max}} \;\;(\text{triangular}) \\ \dfrac{|\Delta \theta|_{eff}}{\omega_{max}} + \dfrac{\omega_{max}}{\alpha_{max}} & \text{otherwise} \;\;(\text{trapezoidal}) \end{cases}$$
   - **Rotating Toward Target ($\omega_{meas} \cdot \Delta \theta \ge 0$):**
     With $\omega_0 = \min(|\omega_{meas}|, \omega_{max})$, $d_{stop, min} = \frac{\omega_0^2}{2 \alpha_{max}}$, $d_{acc} = \frac{\omega_{max}^2 - \omega_0^2}{2 \alpha_{max}}$, $d_{dec} = \frac{\omega_{max}^2}{2 \alpha_{max}}$:
     - _Overshoot recovery ($d_{stop, min} > |\Delta \theta*{net}|$):* $t_{rot} = \frac{\omega_0}{\alpha_{max}} + 2 \sqrt{\frac{d_{stop, min} - |\Delta \theta_{net}|}{\alpha_{max}}}$
     - _Triangular ($d_{stop, min} \le |\Delta \theta*{net}| \le d*{acc} + d*{dec}$):* $\omega_{peak} = \sqrt{\alpha_{max}|\Delta \theta_{net}| + \frac{1}{2}\omega_0^2}, \quad t_{rot} = \frac{2 \omega_{peak} - \omega_0}{\alpha_{max}}$
     - _Trapezoidal ($|\Delta \theta_{net}| > d*{acc} + d*{dec}$):_ $t_{rot} = \frac{\omega_{max} - \omega_0}{\alpha_{max}} + \frac{|\Delta \theta_{net}| - (d_{acc} + d_{dec})}{\omega_{max}} + \frac{\omega_{max}}{\alpha_{max}}$

2. **Compute Linear Speed Ceiling $v_{cap}$ across Approach Corridor $[k_{curr}, k_{anchor}]$:**
   With remaining distance $d_{anchor} = \max(0, s_{anchor} - s)$ and anchor exit speed $v_{target}$ ($0$ for stop, $v_{pass}$ for through):
   $$v_{cap, kin} = \frac{2 \cdot d_{anchor}}{t_{rot} + \epsilon} - v_{target}, \quad v_{cap, brake} = \sqrt{v_{target}^2 + 2 \cdot a_{brake} \cdot d_{anchor}}$$
   $$v_{cap} = \max\left(0.0, \; \min\left(v_{cap, kin}, \; v_{cap, brake}\right)\right)$$
   Apply $v_{lim, k} \leftarrow \min(v_{lim, k}, v_{cap})$ for all $k \in [k_{curr}, k_{anchor}]$ (and $v_{lim, k_{anchor}} \leftarrow \min(v_{lim, k_{anchor}}, v_{target})$) before running `VelocityProfiler`.
3. **Stalled Gate Watchdog:** If a hard gate is held at $d_{anchor} \approx 0$ for $> 1.0\text{ s}$ (e.g. pinned against a bumper), emit `EVENT_HEADING_GATE_STALLED`.

---

## 5. Swerve Wheel Speed & Friction Allocation ($\vec{v}_{module, i} = \vec{v}_{chassis} + \vec{\omega} \times \vec{r}_{module, i}$)

To prevent outer swerve wheels from saturating or losing grip when translating and rotating at the same time, `HeadingPlanner` computes effective translational limits from instantaneous $\omega_{cmd}$ and $\alpha_{cmd} = \frac{\Delta \omega_{ref}}{\Delta t}$ and passes them to `VelocityProfiler` and `VectorLimiter`:
$$v_{max, eff} = \max\left(v_{min}, \; v_{max} - \gamma_{\omega} |\omega_{cmd}| R_{chassis}\right)$$
$$a_{fric, eff} = \sqrt{\max\left(a_{min}^2, \; a_{fric}^2 - \left(\gamma_{\alpha} R_{chassis} \alpha_{cmd}\right)^2\right)}$$
where $R_{chassis} = \sqrt{L_x^2 + L_y^2}$, $v_{min} = 0.25\text{ m/s}$, $a_{min} = 1.00\text{ m/s}^2$, $\gamma_\omega = 0.70$, and $\gamma_\alpha = 0.70$.
