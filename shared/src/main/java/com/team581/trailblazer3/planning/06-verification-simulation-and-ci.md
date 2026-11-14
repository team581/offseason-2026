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
  - Check fillet deviation against the tracking-compensated deviation budget: $d_{dev,k} = r_k (\sec(\theta_k/2) - 1) \le \delta_{geom,k} = \max(0.5\delta_k, \delta_k - \epsilon_{track})$ when geometry permits.
  - Check coupled symmetric tangent allocation on short adjacent legs ($t_{des, k} \le 0.85 \min(L_k, L_{k+1})$ and $t_{clamped, k-1} + t_{clamped, k} \le 0.95 L_k$) for tangency and $G^1$ continuity.
  - Check exact `atan2` midpoint fillet-arc projection near $r_k \to 0$ and large-radius gentle turns ($r_k \to \infty$) without division-by-zero singularities.
  - Check that collinear corners omit fillets, small nonzero turns preserve tangency, and exact reversals require stops ([01](01-path-geometry-and-kinematics.md)).
  - Check that near-reversal corners ($135^\circ \le \theta < 180^\circ$) log a `NEAR_REVERSAL_CORNER` warning without changing `Goal.through` to a stop.
- `VelocityProfilerTest`:
  - Check curvature, join-curvature, and effective steer-rate ($\dot{\delta}_{eff}$) corner limits alongside combined rotation-budget limits per [02](02-velocity-profiler-and-limits.md).
  - Check Pass 1 + Pass 2 preservation of backward braking ($v_{k+1} \le v_{bwd,k+1}$), exact handling of non-binding motor intervals ($g(u_{hi}) \le 0$), 4-step bisection accuracy under resting-$V_{oc}$ and instantaneous supply-current rolloff, and quadratic ($v^2$-linear) lookahead interpolation at `50 Hz` and `250 Hz`.
  - Check lookahead tapering toward the earliest closed gate/stop barrier ($s_{barrier} - s$), `Goal.cruiseThrough` dynamic `d_overrun`, stop samples equaling $0.0$ m/s, terminal 2D pose convergence (including pullback at $s_{meas} \ge s_{barrier}$), and zero velocity chatter under noisy simulated measurements.
- `HeadingPlannerTest`:
  - Check one-shot directional branch latching (`SHORTEST`, `CW`, `CCW`), zero-headroom / saturated-feedforward `NaN`-free arrest ($\dot{e}_{max} = 0$, $\alpha_{eff} = 0$), and continuous unwrapped evolution $\theta_{ref}$ across $\pm\pi$.
  - Check `facePoint` smooth speed-to-target blending and `SINGULAR_HOLD` freezing inside $0.25\text{ m}$ radial distance without $\pi$-flips or angular jitter.
  - Check error-frame `TrapezoidStep` seeded from `lastCommand` $\omega_{prev}$ and `HARD` gate anchoring with `maxGateHoldRadius = 0.25 m` pullback, $|\theta^*_{unwrapped} - \theta_{ref}| < \pi/2$ branch completion, and $2\epsilon_\theta$ open-latch hysteresis ([03](03-heading-and-rotation.md)).
  - Check whole-plan `AutoSegment.forAlliance` under both `ROTATIONAL_180` and `MIRROR_X`, including `offset`, static `facePoint`, `CW <-> CCW` reversal, and `MIRROR_X` keepout polygon winding reversal.
- `VectorLimiterTest`:
  - Check exact 7-constraint 2D active-set projection (1 work-energy origin speed/motor disk, 1 directional braking half-space, 1 isotropic friction-slew disk, and 4 exact field-frame module-speed disks) across all quadrants, concentric-disk ($\omega_{cmd} = 0$) `NaN` immunity, and shrinking-envelope fallback preserving smooth braking when budgets drop mid-path.
- `ProgressTrackerTest`:
  - Check that measured progress follows forward and backward motion within the loop-rate-independent fixed-origin search horizon around $s_{high\_water}$ without jumping across self-intersecting/figure-8 paths, and freezes $s_{high\_water}$ off-corridor or at closed gates.
  - Check 3-tier `Goal.through` capture (`PASSED_WITHIN_TOLERANCE` vs `PASSED_OUTSIDE_TOLERANCE` + multi-goal single-tick traversal), snapshot-based dynamic rebuild deadband (`0.01 m` position), relative `beforeGoal` clamping on relatch, and `activate(plan, startAtGoalId)` pre-populating prior markers/gates cleanly.

### 2.2 Layer 2: Property & Fuzz Testing (Nightly)
Runs randomized path geometries, start states, loop rates (`nominalDtSeconds` in `[4 ms, 20 ms]`), and noise levels; asserts zero `NaN`/`Infinity` outputs, $\|\Delta\vec{v}\|/\Delta t \le a_{fric} + 10^{-6}$ everywhere, $\|\vec{v}_{cmd}\| \le v_{max} + 10^{-6}$ when `!result.infeasible()`, monotonic speed arrest ($\|\vec{v}_{cmd}\| \le \|\vec{v}_{prev}\| - \min(a_{brake}, a_{fric,eff,0})\Delta t + 10^{-6}$) during overspeed `INFEASIBLE` recovery, and bounded completion for feasible scenarios. Infeasible initial states or unreachable goals must report the declared recovery/failure behavior.

### 2.3 Layer 3: Closed-Loop Auto Simulation (Every Push & PR)
Runs dedicated TB3 scenarios (straight/short stops, `Goal.cruiseThrough` score-and-go, corners, reversals, figure-8, heading gates, and dynamic assists) against the swerve physics model ($\tau_{lag}, a_{fric}, a_0, v_{free}$) and asserts:
- Segment duration $t_{actual} \le t_{budget}$.
- Stop goals satisfy their declared position, heading, linear/angular speed, and dwell tolerances.
- `Goal.through` waypoints report `PASSED_WITHIN_TOLERANCE` (`PASSED_OUTSIDE_TOLERANCE` fails CI), `Goal.cruiseThrough` meets terminal velocity/overrun bounds, and swept bumper footprint stays outside `K_hard` keepout zones (logging any `K_margin` encroachment).
- Event markers are recorded as passed once in authored order, including across geometry rebuilds and `activate(plan, startAtGoalId)` starts.

### 2.4 Layer 4: Perturbation Sweeps (20x Monte Carlo per PR)
Runs each auto across $N = 20$ seeded runs with start pose offsets $\mathcal{U}(-0.08, 0.08)\text{ m} / \mathcal{U}(-5^\circ, 5^\circ)$, odometry noise $\sigma = 0.015\text{ m}$, a $0.25\text{ m}$ vision pose step at $t = 3.0\text{ s}$, and a $1.5\text{ m/s}$ ($100\text{ ms}$) lateral collision shove; asserts recovery within the scenario envelope or its expected failure status. Include target loss, disabled/re-enabled operation, driver handoff, and missed ticks.

### 2.5 Layer 5: Log Replay Testing
Feeds `.wpilog` match poses/velocities into the planner and tracks model acceleration residual $`\frac{1}{M}\sum \lVert \vec{a}_{achieved, i} - \vec{a}_{commanded, i}\rVert`$ to investigate model mismatch, tread wear, or battery sag. Replay is open-loop evidence; it cannot prove a changed controller completes the path.

---

## 3. Golden Metrics & PR Checks

Store baselines in `src/test/resources/baselines/auto-metrics.json`. Fail CI if duration increases by $> 0.2\text{ s}$ or keepout clearance drops by $> 0.05\text{ m}$ unless the baseline is explicitly updated.

Baseline updates cannot waive absolute correctness checks. Keep simulation metrics and traces as described in [05](05-developer-tooling.md).

Measure planner latency/allocation on the target 2027 controller (`50–250 Hz`) with worst-case supported plans (target: below $\min(1.0\text{ ms},\; 0.20\,\Delta t_{nom})$ per tick). Keep plant/checker implementations independent and test parameter mismatch; simulated agreement alone does not establish hardware readiness.


## Straight-line acceptance

Calculate an independent rest-to-rest reference from the motor curve, traction, braking and speed ceiling. Compare planned peak speed within 2% and braking position within one 0.05 m spatial sample. Check each command against the resolved limits. Check the 2 m stop within 3 cm, 2 degrees, 0.10 m/s and 0.10 rad/s for 0.10 s, then verify one second of stopped behavior. Run the deterministic plant on both alliances and repeat Phoenix five times per alliance. Keep measured and planned traces and record discrepancies.
