# 06 — Verification Simulation and CI

**Applies To:** `TimeSource`, `AutoSimulationHarness`, `PerturbationTest`, `LogReplay`  

---

## 1. Deterministic Time & State Injection

TB1’s direct `MathSharedStore.getTimestamp()` and `DriverStation.isDisabled()` calls prevented fast-forward tests. Tests supply the [time and state interfaces](04-runtime-integration-and-teleop-assist.md#31-time-and-state-injection) and capture one coherent [TickInput](04-runtime-integration-and-teleop-assist.md#3-tick-contract) per update.

In simulation/CI, step `TimeSource` synthetically (`simTime += dt`) and supply synthetic pose, field-relative speeds, and enabled state through `RobotStateSource`. Seed inputs, require repeatability within documented numerical tolerances, and measure execution time.

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
  - Check fillet deviation: $d_{dev} = r (\sec(\theta/2) - 1)$.
  - Check coupled symmetric tangent allocation on short adjacent legs ($t_{des, k} \le 0.85 \min(L_k, L_{k+1})$ and $t_{clamped, k-1} + t_{clamped, k} \le 0.95 L_k$) for tangency and $G^1$ continuity.
  - Check that collinear corners omit fillets, small nonzero turns preserve tangency, and exact reversals require stops ([01](01-path-geometry-and-kinematics.md)).
  - Check that near-reversal corners ($135^\circ \le \theta < 180^\circ$) log a `NEAR_REVERSAL_CORNER` warning without changing `Goal.through` to a stop.
- `VelocityProfilerTest`:
  - Check curvature and combined motion limits per [02](02-velocity-profiler-and-limits.md), including exact endpoints, interval braking/acceleration, and the final command after feedback/feedforward.
  - Check that stop samples equal $0.0$ m/s, backward braking deceleration obeys $a_{brake}$, and terminal 2D pose convergence eliminates both cross-track and inline residual error.
  - Check second-order lookahead startup from rest ($v_0 = 0 \implies s_{eval, v} = s + \frac{1}{2} a_{avail}(s) \tau_{eff}^2 > s$) and acceleration matching motor torque curve $a_0 (1 - v/v_{free})$.
  - Check for no velocity chatter with noisy simulated measurements.
- `HeadingPlannerTest`:
  - Check rotation direction (`SHORTEST`, `CW`, `CCW`) and continuous unwrapped reference evolution $\theta_{ref}$.
  - Check that hard gates hold at anchors until measured readiness; overshoot/stall must not report successful passage ([03](03-heading-and-rotation.md)).
- `VectorLimiterTest`:
  - Check isotropic 2D acceleration bounds: $`\lVert \vec{v}_{cmd} - \vec{v}_{prev}\rVert \le a_{fric, eff} \cdot \Delta t`$ across all quadrants.
- `ProgressTrackerTest`:
  - Check that measured progress follows forward and backward motion within the search window around $s_{high\_water}$ without jumping across self-intersecting/figure-8 paths.
  - Check that event progress never decreases within a geometry generation.
  - Check that latching preserves interior `Goal.through` vertices and re-latching triggers when $|e_{xt}| > e_{relatch}$ or on explicit localization resets. Rebuilds preserve goal/event identity without reusing old arc-length coordinates.

### 2.2 Layer 2: Property & Fuzz Testing (Nightly)
Runs randomized path geometries, start states, and noise levels; asserts zero `NaN`/`Infinity` outputs, $`\lVert \vec{v}_{cmd}\rVert \le v_{max} + 10^{-6}`$, $`\lVert \Delta \vec{v}\rVert/\Delta t \le a_{fric} + 10^{-6}`$, and bounded completion for feasible scenarios. Infeasible initial states or unreachable goals must report the declared recovery/failure behavior.

### 2.3 Layer 3: Closed-Loop Auto Simulation (Every Push & PR)
Runs dedicated TB2 scenarios (straight/short stops, corners, reversals, figure-8, heading gates, and dynamic assists) against the swerve physics model ($\tau_{lag}, a_{fric}, a_0, v_{free}$) and asserts:
- Segment duration $t_{actual} \le t_{budget}$.
- Stop goals satisfy their declared position, heading, linear/angular speed, and dwell tolerances.
- Through goals pass within declared `within` distance and swept bumper footprint stays outside keepout zones.
- Event markers fire once in authored order, including across geometry rebuilds.

### 2.4 Layer 4: Perturbation Sweeps (20x Monte Carlo per PR)
Runs each auto across $N = 20$ seeded runs with start pose offsets $\mathcal{U}(-0.08, 0.08)\text{ m} / \mathcal{U}(-5^\circ, 5^\circ)$, odometry noise $\sigma = 0.015\text{ m}$, a $0.25\text{ m}$ vision pose step at $t = 3.0\text{ s}$, and a $1.5\text{ m/s}$ ($100\text{ ms}$) lateral collision shove; asserts recovery within the scenario envelope or its expected failure status. Include target loss, disabled/re-enabled operation, driver handoff, and missed ticks.

### 2.5 Layer 5: Log Replay Testing
Feeds `.wpilog` match poses/velocities into the planner and tracks model acceleration residual $`\frac{1}{M}\sum \lVert \vec{a}_{achieved, i} - \vec{a}_{commanded, i}\rVert`$ to investigate model mismatch, tread wear, or battery sag. Replay is open-loop evidence; it cannot prove a changed controller completes the path.

---

## 3. Golden Metrics & PR Checks

Store baselines in `src/test/resources/baselines/auto-metrics.json`. Fail CI if duration increases by $> 0.2\text{ s}$ or keepout clearance drops by $> 0.05\text{ m}$ unless the baseline is explicitly updated.

Baseline updates cannot waive absolute correctness checks. Publish a summary and preview artifacts through the shared runner in [05](05-developer-tooling.md).

Measure planner latency/allocation on the target robot with worst-case supported plans (initial target: below 1 ms per 20 ms tick). Keep plant/checker implementations independent and test parameter mismatch; simulated agreement alone does not establish hardware readiness.
