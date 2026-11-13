# Trailblazer 2 Design Specification: Velocity Profiling & Physical Limits

**Document ID:** TB2-SPEC-02
**Status:** Implementation design
**Author:** Team 581 Technical Staff
**Applies To:** `VelocityProfiler`, `DrivetrainLimits`, `VectorLimiter`

---

## 1. Overview

### How Trailblazer 1 Works

Trailblazer 1 treats a path like a list of points to chase one by one:

1. **Pick a target point:** `HeuristicPathTracker` picks the next waypoint in the list (looking a couple points ahead if waypoints are close together).
2. **Drive straight at it:** `PidPathFollower` points the velocity vector directly at that target point (`getDriveDirection`).
3. **Use a PID loop to pick speed:** It feeds the remaining distance to the end of the path into a PID controller ($kP = 3.5$). Far from the goal, the PID output is huge, so it just gets clamped to `maxVelocity`. Near the goal, the PID output drops off as distance shrinks.
4. **Limit speed-up, ignore slow-down and turning:** `ConstraintsCalculator` limits how fast the robot _accelerates_ (speeding up), but has **no limit on slowing down** for an intermediate waypoint and **no limit on how sharply the velocity direction can snap** when switching from one waypoint to the next.

**Why that causes problems on the robot:**

- **Sharp corner snaps:** When switching from Point A to Point B, the target direction changes instantly. At $4\text{ m/s}$, snapping the velocity direction by $90^\circ$ in one loop requests roughly $283\text{ m/s}^2$ of vector acceleration over a 20 ms loop—so the tires skid and scrub across the carpet.
- **No braking lookahead for corners:** Because the PID loop only cares about distance to the _final_ point, the robot charges into tight mid-path corners at full speed and only tries to slow down _after_ it has already entered the turn.
- **Assumed numbers per auto:** Per-auto speed and acceleration requests (`4.75 m/s`, `8.0 m/s^2`) can exceed the drivetrain’s validated capabilities.

---

### How Trailblazer 2 Works

Instead of chasing one point at a time with a distance PID loop, Trailblazer 2 plans speeds along the **whole upcoming path** using real robot physics:

1. **Round the corners first:** Sharp corners between waypoints are replaced with smooth circular arcs whose closest approach satisfies each waypoint's `within` tolerance.
2. **Slice the path into $5\text{ cm}$ steps:** The upcoming path is broken into $5\text{ cm}$ checkpoints from where the robot is now to where it next stops.
3. **Set speed limits at every step:** Tighter turns automatically get lower speed limits to respect the modeled lateral traction limit ($v \le \sqrt{a_{lat} \cdot r}$). Stop points get a speed limit of $0\text{ m/s}$.
4. **Two quick passes (Backward then Forward):**
   - **Backward pass (Braking):** Starting from the end of the path and working backward, it calculates the fastest speed the robot can be going at each step and still brake in time for upcoming corners and stops.
   - **Forward pass (Accelerating):** Starting from the robot's current speed and working forward, it caps how fast the robot can speed up based on real Kraken X60 motor torque and tire grip.
5. **Follow the curve + smooth 2D vector clamp:** The robot drives along the curve's direction (plus a gentle push back toward the line if bumped off course), and a final 2D limiter ensures the _combined_ speed and direction change in any single loop respects the modeled traction limit for feasible input states.

| Feature                     | Trailblazer 1                                          | Trailblazer 2                                                          |
| :-------------------------- | :----------------------------------------------------- | :--------------------------------------------------------------------- |
| **How speed is chosen**     | Distance-to-end PID loop                               | 2-pass physics profile (braking backward, motor torque forward)        |
| **How corners are handled** | Drive straight at point, snap direction to next point  | Smooth circular turn + automatic slow-down _before_ the turn           |
| **Slowing down mid-path**   | Drops speed limit instantly (infinite braking)         | Smoothly brakes ahead of time using $a_{brake}$                        |
| **Direction changes**       | Only 1D speed is limited; direction can flip instantly | Full 2D vector $(v_x, v_y)$ change is limited by tire grip             |
| **What you tune**           | Per-auto / per-point magic speeds & PID gains          | **Robot-level limits and tracking defaults**, validated on carpet |

---

## 2. Drivetrain Physical Limits Model

Instead of guessing constraints per auto (`LinearConstraintOptions(4.5, 8.0)`), physical properties are defined once for the robot in an immutable record:

```java
public record DrivetrainLimits(
    double maxLinearVelocity,       // v_max (m/s): Derived loaded top speed on carpet
    double freeLinearAcceleration,  // a_0 (m/s^2): Derived stall/zero-speed acceleration
    double motorFreeSpeed,          // v_free (m/s): Derived motor theoretical free speed
    double maxBrakingDeceleration,  // a_brake (m/s^2): On-robot tuned safe braking
    double frictionLimit,           // a_fric (m/s^2): On-robot tuned tire traction (μ * g)
    double maxLateralAcceleration,  // a_lat (m/s^2): Derived cornering limit (0.85 * a_fric)
    double maxAngularVelocity,      // omega_max (rad/s): Derived maximum rotation rate
    double maxAngularAcceleration,  // alpha_max (rad/s^2): Derived maximum rotational acceleration
    double driveResponseLagSeconds  // tau_lag (s): On-robot validated velocity tracking lag
) {}
```

- **Derived Estimates:** Compute initial speed/acceleration limits from CAD, gearing, motor data, and empirical calibration. Validate them on each robot; they are not exact guarantees.
- **Empirical Parameters:** Traction $a_{fric}$, braking $a_{brake}$, and lag $\tau_{lag}$ are primary carpet measurements. Acceleration and braking need not be symmetric.
- **Tracking Policy:** Keep $\tau_{xt}$, $v_{xt,max}$, $kP_\theta$, sampling, and recovery thresholds separate from physical capabilities. Supply module locations through the execution context.
- **Motor Torque-Speed Approximation:**
  $$a_{motor}(v) = a_0 \max\left(0, 1 - \frac{v}{v_{free}}\right)$$

> See **[TB2-SPEC-07: Parameter Derivation & Physics Sandbox Validation](07-empirical-calibration-routine.md)** for derivation estimates, input assumptions, and calibration.

---

## 3. Two-Pass Spatial Horizon Profiler

The complete leg through the next stop is sampled at maximum spacing $\Delta s = 0.05$ m initially ([01](01-path-geometry-and-kinematics.md)).

1. **Include both endpoints:** Uniform sampling needs $N = \lceil S_{horizon}/\Delta s\rceil + 1$ nodes, indexed $0$ through $N-1$.
2. **Insert exact boundaries:** Add current measured progress, piece boundaries, goals, heading anchors, and zone boundaries. Recount $N$ after insertion; use actual interval length $\Delta s_k=s_{k+1}-s_k$.
3. **Handle zero length:** Use terminal position/heading control directly; do not divide by path length.

```
Node:     0        1        2        k       k+1               N-1 (End)
Arc (s):  0.00     0.05     0.10 ... s_k     s_{k+1} ...       S_horizon
          |--------|--------|--------|--------|-----------------|
                   Δs = 0.05m
```

### 3.1 Node-Level Limit Formulation

Use the final angular command and its actual change to obtain $v_{max,eff}$ and $a_{fric,eff}$ from [TB2-SPEC-03 §5](03-heading-and-rotation.md#5-swerve-wheel-speed--friction-allocation). These budgets can reach zero; do not add positive floors that exceed available capacity.

At each spatial sample $k$, the local velocity limit $v_{lim, k}$ is bounded by:

1. **Effective Translational Velocity Limit:** $v_{max, eff}$
2. **Curvature / Centripetal Acceleration Limit:**
   For local curvature magnitude $|\kappa_k|$ and $a_{lat,eff}=\min(a_{lat},a_{fric,eff})$:
   $$v_{corner,k}=\sqrt{\frac{a_{lat,eff}}{|\kappa_k|}}$$
   (On straight segments where $\kappa_k = 0$, $v_{corner,k}=\infty$.)
3. **Zone & Hard Heading Gate Constraints:** Apply zone speed ceilings. An unsatisfied hard gate inserts a zero-speed barrier at its anchor; the backward pass propagates braking upstream ([TB2-SPEC-03 §4](03-heading-and-rotation.md#4-translation-rotation-coupling-hard-vs-soft-gates)).
4. **Goal Exit Constraints:** A stop has $v_{goal,k}=0$. A through goal may declare a speed ceiling $v_{pass}$, never a required minimum; otherwise $v_{goal,k}=\infty$.

The composite speed ceiling is:
$$v_{lim,k}=\min(v_{max,eff},v_{corner,k},v_{zone,k},v_{gate,k},v_{goal,k})$$

### 3.2 Friction Circle Coupling (Tangential Acceleration Capacity)

When cornering at speed $v$, lateral acceleration $a_{lateral} = v^2 |\kappa|$ consumes a portion of the effective translational tire friction ($a_{fric, eff}$).
By the circular friction envelope ($a_{tangential}^2 + a_{lateral}^2 \le a_{fric, eff}^2$), the remaining tangential acceleration capacity is:
$$a_{t}(v, \kappa) = \sqrt{\max\left(0, \; a_{fric, eff}^2 - (v^2 \kappa)^2\right)}$$

### 3.3 Dynamic Programming Sweeps

#### Pass 1: Backward Pass (Enforcing Braking Limits down to $k_{curr}$)

The backward pass propagates braking from the terminal node to $k_{curr}$, the inserted node at current measured progress.

1. **Initialize:** $v_{N-1}=v_{lim,N-1}$.
2. **Bound traction over each interval:** For candidate endpoints, use $\bar v=\max(v_k,v_{k+1})$ and the greatest curvature magnitude $\bar\kappa$ on that interval:
   $$a_{dec,k}=\min\left(a_{brake},a_t(\bar v,\bar\kappa)\right)$$
3. **Solve backward, $k=N-2$ down to $k_{curr}$:** Choose the largest nonnegative $v_k$ satisfying both:
   $$v_k\le v_{lim,k},\qquad v_k^2\le v_{k+1}^2+2a_{dec,k}\Delta s_k$$

**Interval check:** Evaluating traction only at the slower downstream endpoint overestimates available braking on a curve. Use a bounded scalar solve or conservative interval bound, and check the resulting interval independently.

#### Pass 2: Forward Pass (Enforcing Motor Acceleration Limits from Current Node $k_{curr}$)

1. **Seed current progress:**
   $$v_{k_{curr}}\leftarrow\min(v_{k_{curr}},v_{start})$$
   If measured motion exceeds the backward envelope, report `INFEASIBLE` and use bounded recovery (§3.4); clamping the profile does not make the measured state feasible.
2. **Bound acceleration over each interval:**
   $$a_{acc,k}=\max\left(0,\min\left(a_{motor}(\bar v),a_t(\bar v,\bar\kappa)\right)\right)$$
3. **Solve forward, $k=k_{curr}$ through $N-2$:** Choose the largest nonnegative $v_{k+1}$ satisfying its existing ceiling and:
   $$v_{k+1}^2\le v_k^2+2a_{acc,k}\Delta s_k$$

**Interval check:** Evaluating motor torque only at the slower upstream endpoint overestimates acceleration. Use actual interval lengths and bounded solving with explicit failure handling in both passes.

```mermaid
flowchart LR
    Init["1. Initialize Limits<br/>Sample horizon nodes and compute local speed limits v_lim"] -->
    Back["2. Backward Pass (Braking)<br/>Solve interval braking bounds backwards"] -->
    Fwd["3. Forward Pass (Acceleration)<br/>Solve interval acceleration bounds forwards"] -->
    Out["4. Output Profile<br/>Interval-checked profile and feasibility diagnostics"]
```

### 3.4 Initial Velocity Selection and Chatter Elimination

1. **Nominal tracking:** Project the previous applied velocity onto the current path tangent:
   $$v_{start}=\max(0,\vec v_{applied}\cdot\hat t(s))$$
   Total vector magnitude would incorrectly count sideways correction as forward speed.
2. **Activation / missing applied command:** Seed from measured velocity. If integration only reports the requested command, label it as such and monitor measured divergence.
3. **Disturbance detection:** Compare vectors, so opposite or sideways motion is detected even at equal speed:
   $$\|\vec v_{meas}-\vec v_{applied}\|>v_{diverge,threshold}$$
   The tracking policy owns threshold/dwell and reseeding; preserve event identity during recovery.
4. **Infeasible motion:** Negative forward velocity needs bounded arrest. Motion above the braking envelope emits `INFEASIBLE`; brake/recover without declaring a stop or gate successfully reached.

---

## 4. Latency Compensation (Lookahead Advance)

To compensate for CAN, motor filter, and steering response lag ($\tau_{lag} \approx 0.040\text{ s}$) without getting stuck at $v = 0$ from rest, `VelocityProfiler` evaluates speed and tangent direction at decoupled lookahead distances:

1. **Tapering Near Stop Goals (No Endpoint Chatter):**
   Using a conservative available deceleration $a_{dec}>0$, within $d_{brake} = \max\left(0.30\text{ m}, \; \frac{v^2}{2 a_{dec}}\right)$ of a stop goal, lookahead tapers smoothly to zero:
   $$\tau_{effective} = (\Delta t + \tau_{lag}) \cdot \min\left(1.0, \; \frac{S_{horizon} - s}{d_{brake}}\right)$$
2. **Second-Order Speed Lookahead ($s_{eval, v}$):**
   $$a_{avail}(s)=\min(a_{motor}(v(s)),a_t(v(s),\kappa(s)))$$
   $$s_{eval, v} = s + v(s) \cdot \tau_{effective} + \frac{1}{2} a_{avail}(s) \cdot \tau_{effective}^2$$
   At rest with positive available acceleration, the second-order term advances the evaluation point and avoids a zero-speed startup deadlock. Final command limits still apply.
3. **Half-Lead Tangent Lookahead ($s_{eval, t}$):**
   Full speed lookahead can turn into corners early. Start with half-lead tangent evaluation and validate it in simulation/carpet tests:
   $$s_{eval, t} = s + v(s) \cdot \left(\frac{1}{2} \Delta t + \frac{1}{2} \tau_{lag}\right)$$
   - **Profile speed $v_{profile}$:** Linearly interpolated at $s_{eval, v}$.
   - **Path unit tangent $\hat{t}_{eval}$:** Evaluated at $s_{eval, t}$.
   - **Path unit normal $\hat{n}(s)$:** Evaluated at measured projection $s$.
4. **Bounds:** Clamp both lookaheads to the leg and any closed gate. Lookahead never advances event progress or samples beyond a required stop. With no braking capacity, enter infeasible recovery rather than divide by zero.

---

## 5. Cross-Track Control, Stop Convergence & 2D Vector Limiter

1. **Cross-Track Feedback Velocity ($v_{xt}$):**
   Since $e_{xt} = (\mathbf{p}_{robot} - \mathbf{p}^*(s)) \cdot \hat{n}(s) > 0$ when left of the path, negative feedback pushes the robot back onto the line:
   $$v_{xt, limit} = \min(v_{xt, max}, \; v_{max, eff}), \quad v_{xt} = \mathrm{clamp}\left(-\frac{e_{xt}}{\tau_{xt}}, \; -v_{xt, limit}, \; v_{xt, limit}\right)$$
2. **Combine Forward and Cross-Track Demand:**
   $$\vec v_{des}=v_{profile}\hat t_{eval}+v_{xt}\hat n(s)$$
   The lookahead tangent and current normal need not be perpendicular. Apply speed/module limits to their actual sum, including any target feedforward; $\sqrt{v_{max,eff}^2-v_{xt}^2}$ alone is not sufficient.
3. **Terminal 2D Convergence at Stop Goals:**
   Because $v_{xt} \hat{n}(s)$ only pushes perpendicular to $\hat{t}$, any residual inline lag when $s$ reaches $S_{horizon}$ would remain uncorrected. Near a `Goal.stop`, blend into direct position convergence using $d_{stop}=\|\mathbf p_{stop}-\mathbf p_{robot}\|$. Choose blend distance/hysteresis in tracking policy:
   $$\vec{e}_{stop} = \mathbf{p}_{stop} - \mathbf{p}_{robot}, \quad v_{term} = \min\left(v_{max, eff}, \; \sqrt{2 \cdot a_{dec} \cdot d_{stop}}, \; \frac{d_{stop}}{0.85 \cdot \tau_{xt}}\right)$$
   $$\vec{v}_{des}=\begin{cases}v_{term}\vec e_{stop}/d_{stop}&d_{stop}>0\\0&d_{stop}=0\end{cases}$$
   Reuse this controller to hold an interior hard gate. Completion requires measured pose/speed/heading and dwell, not zero profile speed ([00](00-overview-and-architecture.md)).
4. **Isotropic 2D Vector Acceleration Clamp (`VectorLimiter`):**
   Bound the 2D velocity change from the previous applied command $\vec{v}_{prev}$ to $\Delta v_{max} = a_{fric, eff} \cdot \Delta t$:
   $$\Delta \vec{v} = \vec{v}_{des} - \vec{v}_{prev}, \quad \vec{v}_{cmd} = \begin{cases} \vec{v}_{prev} + \Delta \vec{v} \cdot \dfrac{\Delta v_{max}}{\|\Delta \vec{v}\|} & \text{if } \|\Delta \vec{v}\| > \Delta v_{max} \\ \vec{v}_{des} & \text{otherwise} \end{cases}$$

5. **Intersect Final Constraints:** Select the closest desired command satisfying the slew bound, speed/module limits, and motor/braking bounds together. Sequential speed and slew clamps can invalidate each other.
6. **Shrinking Envelope:** If no reachable command satisfies all new bounds, emit `INFEASIBLE`, reduce angular demand where possible, and apply bounded braking toward the admissible region. Do not claim simultaneous instantaneous bounds during recovery. Disabled/emergency output inhibition takes precedence.

---

## 6. Execution Budget & Telemetry

- **Bounded Buffers:** Size profile arrays for validated plan capacity, both endpoints, and inserted boundaries. Reject overflow instead of dropping constraints. Reuse buffers where profiling warrants it.
- **Measured Performance:** Benchmark worst-case runtime and allocation on the target robot; acceptance evidence belongs in [06](06-verification-simulation-and-ci.md), not an unmeasured microsecond claim.
- **Limiting Factor Logging:** Every loop logs `Trailblazer/LimitingFactor` to DogLog so AdvantageScope shows why the robot ran at its commanded speed:
  ```java
  public enum LimitingFactor {
      TOP_SPEED, CORNER_CENTRIPETAL, ZONE_SPEED_CAP, GOAL_DECELERATION,
      MOTOR_ACCELERATION, HEADING_HARD_GATE, VECTOR_SLEW_LIMIT, MODULE_SPEED, INFEASIBLE
  }
  ```

Log profile and final-command constraints separately; more than one limit may be active.
