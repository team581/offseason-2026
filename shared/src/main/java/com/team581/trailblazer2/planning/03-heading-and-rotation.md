# Trailblazer 2 Design Specification: Holonomic Heading Scheduling & Coupling

**Document ID:** TB2-SPEC-03  
**Status:** Approved for Implementation  
**Author:** Team 581 Technical Staff  
**Applies To:** `HeadingPlanner`, `HeadingKeyframe`, `HeadingGate`  

---

## 1. Problem Statement: Deficiencies of Trailblazer 1

Trailblazer 1 attempted to handle holonomic rotation through an ad-hoc set of heuristics:

1. **Heading Coupled to Waypoint Points:** The robot's rotation target was tied to waypoints rather than decoupled as an independent schedule.
2. **Abrupt Arc Midpoint Flipping:** For curved paths, `HeuristicPathTracker` injected an intermediate midpoint rotation that abruptly switched targets at $t = 0.5$:
   ```java
   targetRotation = t < 0.5 ? midRotation : targetPose.getRotation();
   ```
   At the exact midpoint of a high-speed curve, the PID setpoint made a step change, causing angular lag, overshoot, and heading oscillation.
3. **Heuristic Translation Pacing:** `PidPathFollower` applied two overlapping velocity caps based on estimated "angular work remaining":
   - Imposed an arbitrary minimum speed floor ($0.4$ m/s) to prevent the robot from stalling under carpet friction.
   - Forced speed to zero if within linear tolerance but outside angular tolerance.
   - Used a default angular tolerance of $360^\circ$ on most points, which inadvertently disabled the rotational cap entirely.
4. **Ambiguous Turn Direction:** Rotation was resolved purely by shortest-path angular modulus. If an intake was extended or a cable track had wind-up limits, the robot could not be commanded to rotate explicitly clockwise or counter-clockwise.

Trailblazer 2 replaces this with a **decoupled heading schedule** over path arc length $s$, supporting explicit rotational direction, continuous feedforward, and rigorous **hard and soft gating**.

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
    AnchorPosition anchor,      // Spatial location along the path
    HeadingTarget target,       // Target orientation rule
    TurnDirection direction,    // Rotation direction constraint
    GatePolicy gate,            // Impact on translational speed
    PoseErrorTolerance tolerance // Arrival tolerance (e.g. ±2.5 deg)
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

*Why this matters in FRC:* If the robot is at $0^\circ$ and needs to face $175^\circ$ while intaking balls from the right side, a `CCW` constraint ensures the intake sweeps toward the balls rather than spinning the back bumpers across the game pieces.

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

### 3.2 Rotational Command Formulation
Every loop, the rotational controller calculates:
$$\omega_{cmd} = \omega_{ref} + kP_\theta \cdot \mathrm{angleModulus}(\theta_{ref} - \theta)$$
- $\omega_{ref}$ provides 100% feedforward matching physical acceleration capabilities.
- Proportional feedback $kP_\theta$ acts as a disturbance compensator.
- **Result:** Module steering scrub is minimized, and heading tracks without phase lag.

---

## 4. Translation-Rotation Coupling: Hard vs. Soft Gates

The core challenge of holonomic navigation is deciding when rotation takes precedence over translation.

```
+-------------------------------------------------------------------------+
|                              GATE POLICIES                              |
|                                                                         |
|  SOFT GATE:                                                             |
|  - Rotate as fast as limits allow (ω_max, α_max).                       |
|  - Translation runs at full profile speed; never slowed down.           |
|  - Best for: Transitions, intake staging, traveling across field.       |
|                                                                         |
|  HARD GATE:                                                             |
|  - Heading MUST be within tolerance upon arrival at the anchor.         |
|  - Translation velocity is capped: v_cap = d_anchor / t_rot.            |
|  - If robot reaches anchor and heading is not ready, translation stops. |
|  - Best for: Scoring shots, intake acquisitions, precise docking.       |
+-------------------------------------------------------------------------+
```

### 4.1 Soft Gate Policy
For a `soft` gate:
- Translation velocity $v_{profile}$ is unaffected by heading error.
- The robot accelerates and corners at maximum linear speed, while rotating as quickly as $\alpha_{max}$ and $\omega_{max}$ permit.

### 4.2 Hard Gate Policy: Analytical Speed Capping
For a `hard` gate anchored at path progress $s_{anchor}$:
The robot must arrive at $s_{anchor}$ with $|\theta - \theta^*| \le \epsilon_\theta$.

1. **Calculate Remaining Rotation Time $t_{rot}$:**
   The time required for a trapezoidal angular profile to traverse $|\Delta \theta| - \epsilon_\theta$ from current angular speed $\omega_{meas}$ to rest is solved analytically:
   $$t_{acc} = \frac{\omega_{max} - |\omega_{meas}|}{\alpha_{max}}$$
   $$d_{acc} = |\omega_{meas}| t_{acc} + \frac{1}{2} \alpha_{max} t_{acc}^2$$
   $$d_{dec} = \frac{\omega_{max}^2}{2 \alpha_{max}}$$
   If total angular distance $|\Delta \theta| \le d_{acc} + d_{dec}$, the profile is triangular:
   $$\omega_{peak} = \sqrt{\alpha_{max} |\Delta \theta| + \frac{1}{2} \omega_{meas}^2}$$
   $$t_{rot} = \frac{\omega_{peak} - |\omega_{meas}|}{\alpha_{max}} + \frac{\omega_{peak}}{\alpha_{max}}$$
   Otherwise:
   $$t_{cruise} = \frac{|\Delta \theta| - (d_{acc} + d_{dec})}{\omega_{max}}$$
   $$t_{rot} = t_{acc} + t_{cruise} + \frac{\omega_{max}}{\alpha_{max}}$$

2. **Calculate Distance to Anchor:**
   $$d_{anchor} = \max\left(0, \; s_{anchor} - s\right)$$

3. **Compute Linear Velocity Ceiling:**
   To guarantee the robot takes at least $t_{rot}$ seconds to reach $s_{anchor}$:
   $$v_{cap} = \frac{d_{anchor}}{t_{rot}}$$

4. **Inject into Velocity Profiler:**
   The velocity profiler treats $v_{cap}$ as a local speed ceiling at $s$.
   - As $d_{anchor} \to 0$, if heading is aligned, $t_{rot} \to 0$ and $v_{cap}$ lifts.
   - If heading is delayed, $v_{cap}$ smoothly slows the robot down to a standstill exactly at the anchor:
     $$d_{anchor} \le \text{tolerance} \implies v_{cap} = 0.0$$
   - The robot halts cleanly at the anchor, waits for rotation to clear, and then proceeds.

### 4.3 Stalled Gate Protection
If a hard gate is blocked from achieving tolerance (e.g. robot physically pinned or wedged against another bumper), a configurable watchdog timer ($t_{timeout} = 1.0$ s) triggers:
- Raises `EVENT_HEADING_GATE_STALLED`.
- Telemetry alerts the driver station.
- User-defined recovery hook in `BaseImperativeAuto` can abort, re-latch, or force gate clearance.

---

## 5. Swerve Traction Coupling (Wheel Speed Allocation)

Holonomic swerve modules share motor power and tire friction between translation and rotation:
$$\vec{v}_{module, i} = \vec{v}_{chassis} + \vec{\omega} \times \vec{r}_{module, i}$$

```
Module Velocity Vector:
                v_chassis
          ----------------------->
          \                      |
           \                     |
   v_module \                    |  ω x r_i
             \                   | (Rotational Vector)
              \                  v
```

If the combined demand exceeds maximum module wheel speed $v_{wheel, max}$:
1. **CTRE / WPILib Desaturation:** Downstream in `Swerve.java`, module speeds are normalized.
2. **Trailblazer Upstream Friction Budget:**
   To prevent module saturation from breaking carpet traction, Trailblazer 2 partitions the friction circle:
   $$a_{trans, max} = \sqrt{\max\left(0, \; a_{fric}^2 - (R_{robot} \cdot \alpha_{max})^2\right)}$$
   where $R_{robot}$ is the radius from robot center to the outermost swerve module.
   By designing $\alpha_{max}$ to consume at most $25\%$ of total tire friction, translational agility is preserved under full spin rates.
