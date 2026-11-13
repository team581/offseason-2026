# Trailblazer 2 Design Specification: Verification, Simulation & CI

**Document ID:** TB2-SPEC-06  
**Status:** Implementation design
**Author:** Team 581 Technical Staff  
**Applies To:** `TimeSource`, `AutoSimulationHarness`, `PerturbationTest`, `LogReplay`  

---

## 1. Deterministic Time & State Injection

Trailblazer 1 called `MathSharedStore.getTimestamp()` and `DriverStation.isDisabled()` directly, preventing fast-forward unit testing. Robot integration injects time and state; calculators receive explicit values and `dt`. Capture one coherent [TickInput](00-overview-and-architecture.md) per update. Example adapter interfaces:

```java
@FunctionalInterface
public interface TimeSource {
    double getTimestampSeconds();
}

public interface RobotStateSource {
    Pose2d getMeasuredPose();
    ChassisSpeeds getMeasuredSpeeds(); // Field-relative
    boolean isEnabled(); // Auto and teleop assists
}
```

- **On Robot:** `TimeSource` binds to `RobotController.getFPGATime() / 1e6`.
- **In Sim & CI:** `TimeSource` steps synthetically (`simTime += dt`), with seeded inputs and repeatable results within documented numerical tolerances. Measure execution time rather than assuming it.

---

## 2. Five-Layer Testing Pyramid

```
                       / \
                      /   \
                     / Log \
                    /Replay \
                   /---------\
                  /Perturbation\
                 /  Sim Sweeps  \
                /----------------\
               / Closed-Loop Sim  \
              /--------------------\
             / Property & Fuzzing   \
            /------------------------\
           /     Fast Unit Tests      \
          /----------------------------\
```

### 2.1 Layer 1: Unit Tests (Every Push & PR)
- `PathBuilderTest`:
  - Validates fillet deviation formula: $d_{dev} = r (\sec(\theta/2) - 1)$.
  - Verifies coupled symmetric tangent length allocation on short adjacent legs ($t_{des, k} \le 0.85 \min(L_k, L_{k+1})$ and $t_{clamped, k-1} + t_{clamped, k} \le 0.95 L_k$) so circular tangency and $G^1$ continuity are preserved.
  - Asserts that collinear corners omit fillets cleanly, small nonzero turns preserve tangency, and exact reversals require stops ([01](01-path-geometry-and-kinematics.md)).
  - Asserts that near-reversal corners ($135^\circ \le \theta < 180^\circ$) log a `NEAR_REVERSAL_CORNER` warning without silently mutating `Goal.through` into a stop.
- `VelocityProfilerTest`:
  - Checks curvature and combined motion limits per [02](02-velocity-profiler-and-limits.md), including exact endpoints, interval braking/acceleration, and the final command after feedback/feedforward.
  - Asserts that a stop goal sample is pinned to $0.0$ m/s, backward braking deceleration obeys $a_{brake}$, and terminal 2D pose convergence eliminates both cross-track and inline residual error.
  - Verifies second-order lookahead startup from rest ($v_0 = 0 \implies s_{eval, v} = s + \frac{1}{2} a_{avail}(s) \tau_{eff}^2 > s$) and acceleration matching motor torque curve $a_0 (1 - v/v_{free})$.
  - Verifies that velocity chatter is eliminated when simulated measured velocity has noise.
- `HeadingPlannerTest`:
  - Verifies rotation direction disambiguation (`SHORTEST`, `CW`, `CCW`) and continuous unwrapped reference evolution $\theta_{ref}$.
  - Asserts that hard gates hold at their anchors until measured readiness, and that overshoot/stall never reports successful passage ([03](03-heading-and-rotation.md)).
- `VectorLimiterTest`:
  - Asserts isotropic 2D acceleration bounds: $\|\vec{v}_{cmd} - \vec{v}_{prev}\| \le a_{fric, eff} \cdot \Delta t$ across all quadrants.
- `ProgressTrackerTest`:
  - Asserts monotonic progress ratcheting ($s_{k+1} \ge s_k$) within the windowed local search interval around $s_{high\_water}$ (preventing loop-jumping on self-intersecting / figure-8 paths).
  - Verifies Waypoint-Vertex Sub-Segment Latching so interior `Goal.through` vertices remain invariant and re-latching triggers cleanly when $|e_{xt}| > e_{relatch}$ or on explicit localization resets. Rebuilds preserve goal/event identity without reusing old arc-length coordinates.

### 2.2 Layer 2: Property & Fuzz Testing (Nightly)
Runs randomized path geometries, start states, and noise levels; asserts zero `NaN`/`Infinity` outputs, $\|\vec{v}_{cmd}\| \le v_{max} + 10^{-6}$, $\|\Delta \vec{v}\|/\Delta t \le a_{fric} + 10^{-6}$, and bounded completion for feasible scenarios. Infeasible initial states or unreachable goals must report the declared recovery/failure behavior.

### 2.3 Layer 3: Closed-Loop Auto Simulation (Every Push & PR)
Runs dedicated TB2 scenarios (straight/short stops, corners, reversals, figure-8, heading gates, and dynamic assists) against the swerve physics model ($\tau_{lag}, a_{fric}, a_0, v_{free}$) and asserts:
- Segment duration $t_{actual} \le t_{budget}$.
- Stop goals satisfy their declared position, heading, linear/angular speed, and dwell tolerances.
- Through goals pass within declared `within` distance and swept bumper footprint stays outside keepout zones.
- Event markers fire once in authored order, including across geometry rebuilds.

### 2.4 Layer 4: Perturbation Sweeps (20x Monte Carlo per PR)
Runs each auto across $N = 20$ seeded runs with start pose offsets $\mathcal{U}(-0.08, 0.08)\text{ m} / \mathcal{U}(-5^\circ, 5^\circ)$, odometry noise $\sigma = 0.015\text{ m}$, a $0.25\text{ m}$ vision pose step at $t = 3.0\text{ s}$, and a $1.5\text{ m/s}$ ($100\text{ ms}$) lateral collision shove; asserts recovery within the scenario envelope or its expected failure status. Include target loss, disabled/re-enabled operation, driver handoff, and missed ticks.

### 2.5 Layer 5: Log Replay Testing
Feeds `.wpilog` match poses/velocities into the planner and tracks model acceleration residual $\frac{1}{M}\sum \|\vec{a}_{achieved, i} - \vec{a}_{commanded, i}\|$ to investigate model mismatch, tread wear, or battery sag. Replay is open-loop evidence; it cannot prove a changed controller completes the path.

---

## 3. Golden Metrics & PR Checks

Stored in `src/test/resources/baselines/auto-metrics.json`. If a PR increases an auto's duration by $> 0.2\text{ s}$ or reduces keepout clearance by $> 0.05\text{ m}$, CI fails unless the baseline is explicitly updated. CI publishes a summary and preview artifacts using the shared runner in [05](05-spec-model-dsl-and-tooling.md). Baseline updates cannot waive absolute correctness checks.

Measure planner latency/allocation on the target robot with worst-case supported plans (initial target: below 1 ms per 20 ms tick). Keep plant/checker implementations independent and test parameter mismatch; simulated agreement alone does not establish hardware readiness.
