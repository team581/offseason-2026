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
|  - Translation velocity is capped by bounded-deceleration & braking envelopes. |
|  - If robot reaches anchor and heading is not ready, translation stops. |
|  - Best for: Scoring shots, intake acquisitions, precise docking.       |
+-------------------------------------------------------------------------+
```

### 4.1 Soft Gate Policy
For a `soft` gate:
- Translation velocity $v_{profile}$ is unaffected by heading error.
- The robot accelerates and corners at maximum linear speed, while rotating as quickly as $\alpha_{max}$ and $\omega_{max}$ permit.

### 4.2 Hard Gate Policy: Analytical Speed Capping & Pacing
For a `hard` gate anchored at path progress $s_{anchor}$:
The robot must arrive at $s_{anchor}$ with $|\theta - \theta^*| \le \epsilon_\theta$.

1. **Calculate Remaining Rotation Time $t_{rot}$:**
   Let $\Delta \theta$ be the signed angular difference to the target: $\Delta \theta = \mathrm{angleModulus}(\theta^* - \theta)$. (If a specific direction is forced via `TurnDirection.CW` or `CCW`, $\Delta \theta$ is unwrapped along that turn direction).
   
   The net angular distance to traverse after accounting for arrival tolerance $\epsilon_\theta$ is:
   $$|\Delta \theta_{net}| = \max\left(0.0, \; |\Delta \theta| - \epsilon_\theta\right)$$
   If $|\Delta \theta_{net}| = 0$, $t_{rot} = 0.0$ (heading is already satisfied).

   **Directional Alignment Check (Reversal Handling):**  
   If the robot is currently rotating in the direction *opposite* to the desired displacement ($\omega_{meas} \cdot \Delta \theta < 0$):
   The robot must first decelerate to zero angular speed before reversing toward $\theta^*$:
   $$t_{turn} = \frac{|\omega_{meas}|}{\alpha_{max}}$$
   $$d_{turn} = \frac{\omega_{meas}^2}{2 \alpha_{max}}$$
   The effective angular distance from rest becomes:
   $$|\Delta \theta|_{eff} = |\Delta \theta_{net}| + d_{turn}$$
   The time required to traverse $|\Delta \theta|_{eff}$ from rest ($\omega_0 = 0$) using trapezoidal / triangular motion is:
   $$t_{profile} = \begin{cases} 
   2 \sqrt{\frac{|\Delta \theta|_{eff}}{\alpha_{max}}} & \text{if } |\Delta \theta|_{eff} \le \frac{\omega_{max}^2}{\alpha_{max}} \quad (\text{triangular}) \\ 
   \frac{|\Delta \theta|_{eff}}{\omega_{max}} + \frac{\omega_{max}}{\alpha_{max}} & \text{otherwise} \quad (\text{trapezoidal}) 
   \end{cases}$$
   Total rotation time:
   $$t_{rot} = t_{turn} + t_{profile}$$

   **Aligned Rotation ($\omega_{meas} \cdot \Delta \theta \ge 0$):**  
   If the robot is already rotating toward the target:
   $$t_{acc} = \frac{\omega_{max} - |\omega_{meas}|}{\alpha_{max}}$$
   $$d_{acc} = \frac{\omega_{max}^2 - \omega_{meas}^2}{2 \alpha_{max}}$$
   $$d_{dec} = \frac{\omega_{max}^2}{2 \alpha_{max}}$$
   If $|\Delta \theta_{net}| \le d_{acc} + d_{dec}$ (triangular profile):
   $$\omega_{peak} = \sqrt{\alpha_{max} |\Delta \theta_{net}| + \frac{1}{2} \omega_{meas}^2}$$
   $$t_{rot} = \frac{\omega_{peak} - |\omega_{meas}|}{\alpha_{max}} + \frac{\omega_{peak}}{\alpha_{max}}$$
   Otherwise (trapezoidal profile with cruise at $\omega_{max}$):
   $$t_{cruise} = \frac{|\Delta \theta_{net}| - (d_{acc} + d_{dec})}{\omega_{max}}$$
   $$t_{rot} = t_{acc} + t_{cruise} + \frac{\omega_{max}}{\alpha_{max}}$$

2. **Calculate Distance to Anchor:**
   $$d_{anchor} = \max\left(0, \; s_{anchor} - s\right)$$

3. **Compute Linear Velocity Ceiling (Bounded Deceleration & Braking Envelopes):**
   A naive constant-velocity formula ($v = d / t$) ignores braking limits and causes high-speed overshoot when heading lags. Trailblazer 2 solves this via two coupled physical envelopes:
   
   - **Kinematic Average-Velocity Envelope:** Under bounded linear deceleration, average velocity to reach exit speed $v_{target}$ (where $v_{target} = 0.0\text{ m/s}$ for a stop goal, or $v_{pass}$ for a through goal) over duration $t_{rot}$ is $\bar{v} = \frac{v_{cap} + v_{target}}{2}$. Equating $d_{anchor} = \bar{v} \cdot t_{rot}$:
     $$v_{cap, kin} = \frac{2 \cdot d_{anchor}}{t_{rot} + \epsilon} - v_{target}$$
   - **Traction Braking Envelope:** Commanded velocity must never exceed what physical braking deceleration $a_{brake}$ can safely arrest within the remaining distance:
     $$v_{cap, brake} = \sqrt{v_{target}^2 + 2 \cdot a_{brake} \cdot d_{anchor}}$$
   - **Composite Velocity Ceiling:**
     $$v_{cap} = \max\left(0.0, \; \min\left(v_{cap, kin}, \; v_{cap, brake}\right)\right)$$

4. **Corridor-Wide Pacing & Backward Profiler Propagation:**
   Applying $v_{cap}$ only at the anchor node $k_{anchor}$ would allow the robot to cruise at full speed and slam the brakes at the last second, arriving prematurely and lingering at rest while rotation completes.
   
   To achieve smooth, synchronous arrival:
   1. **Approach Corridor Pacing:** $v_{cap}$ caps the maximum allowable linear speed across all nodes from current progress up to the anchor:
      $$v_{lim, k} \leftarrow \min\left(v_{lim, k}, \; v_{cap}\right) \quad \forall k \in [0, \; k_{anchor}]$$
      This forces both the backward and forward profiler passes to pace translation smoothly across the approach interval so that transit duration $\Delta t_{transit} \approx t_{rot}$.
   2. **Anchor Boundary Condition:** At the anchor node itself, $v_{lim, k_{anchor}} = \min(v_{lim, k_{anchor}}, v_{target})$.
   3. **Dynamic Unlocking:**
      - If heading finishes early ($t_{rot} \to 0$), $v_{cap, kin}$ rises, lifting the pacing cap and allowing translation to proceed at full speed without hesitation.
      - If heading is delayed, $v_{cap}$ drops. As $d_{anchor} \to 0$, $v_{cap} \to 0$, bringing the robot smoothly to rest at $s_{anchor}$ until the heading tolerance gate clears.

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
