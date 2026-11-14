# 07 — Empirical Calibration Routine

**Applies To:** `DrivetrainLimits`, `RobotDrivetrainSpec`, `DrivetrainLimitsCalculator`, 2027 robot Java configuration

---

## 1. Overview & Principles

Keep physical parameters and per-waypoint PID gains out of path definitions:

- **Autos declare operational intent and geometric tolerances** (e.g. `within(0.25)`, `limits: fast` $= 0.90 \times \text{limits}$, `limits: careful` $= 0.60 \times \text{limits}$).

- **Physical capabilities belong to the drivetrain**, stored in robot Java configuration. Start with CAD/motor estimates and derived controller defaults; tune three primary empirical parameters ($a_{fric}, a_{brake}, \tau_{lag}$) during bringup on the new 2027 robot.

---

## 2. Mathematically Derived vs. On-Robot Tuned Parameters

Derive constants from CAD, gearing, and motor datasheets / WPILib `DCMotor` models; measure carpet and closed-loop latency properties on the new 2027 robot.

### 2.1 Category A: Derived Parameters and Initial Controller Estimates

Illustrative testbed inputs below are not defaults for the new 2027 robot; supply each robot's own CAD geometry, module layout, and motor constants via `RobotDrivetrainSpec`.

Inputs (`RobotDrivetrainSpec`):

- CAD geometry & drivetrain layout: drive/steer gear ratios $G_{drive}, G_{steer}$, loaded wheel radius $r_{wheel}$, module positions $(x_i, y_i) \implies R_{max} = \max_i \sqrt{x_i^2 + y_i^2}$, robot mass $m$, yaw MOI $J_{zz}$, stator slip limit $I_{slip}$, nominal battery resistance $R_{bat} \approx 0.018\text{ }\Omega$, static friction $k_S \approx 0.20\text{ V}$.
- Drive/steer motor models via `DCMotor driveMotor` and `DCMotor steerMotor` (which already carry exact $R_m, k_T, k_V, I_{free}, \omega_{free}$ for whichever motor/commutation mode the 2027 robot uses; illustrative testbed Kraken X60 FOC constants: $V_{nom} = 12\text{ V}$, $\omega_{free} = 607.37\text{ rad/s}$, $\tau_{stall} = 9.37\text{ N}\cdot\text{m}$, $I_{stall} = 483\text{ A}$, $I_{free} = 2.0\text{ A}$, $R_m = 12/I_{stall} = 0.0248\text{ }\Omega$, $k_T = \tau_{stall}/I_{stall} = 0.0194\text{ N}\cdot\text{m/A}$, $k_V = (12 - I_{free}R_m)/\omega_{free} = 0.01967\text{ V/(rad/s)}$).
- Peak current & baseline derates: $I_{max} = \min\left(I_{slip},\; (V_{oc} - k_S)/(R_m + 4 R_{bat}/\eta_{inv})\right)$ with baseline derates $\eta_{inv} \approx 0.95$, $\eta_{gear} \approx 0.90$, $\eta_a \approx 0.90$, $\eta_\omega \approx 0.85$, $\eta_\alpha \approx 0.85$, and nominal loop period $\Delta t_{nom}$.

| Parameter                        |     Symbol     | Units  | Mathematical Derivation Formula                                                                                                                               | Required Input Constraints                                                                                     |
| :------------------------------- | :------------: | :----: | :------------------------------------------------------------------------------------------------------------------------------------------------------------ | :------------------------------------------------------------------------------------------------------------- |
| **1. Motor Free Speed**          |   $v_{free}$   |  m/s   | $\dfrac{V_{nom} - k_S}{k_V \cdot G_{drive}} \cdot r_{wheel}$                                                                                                  | `driveMotor` ($k_V$), $G_{drive}$, $r_{wheel}$, $k_S$                                                          |
| **2. Loaded Top Speed**          |   $v_{max}$    |  m/s   | $\eta_v \cdot v_{free}$ ($\eta_v \approx 0.90\text{–}0.92$)                                                                                                   | $v_{free}$, battery sag & steering cosine headroom                                                             |
| **3. Stall Acceleration**        |     $a_0$      |  m/s²  | $\eta_a \min\left(a_{0,raw},\; a_{fric}\right)$, where $a_{0,raw} = \dfrac{4 \eta_{gear} G_{drive} k_T (I_{max} - I_{free})}{m \cdot r_{wheel}}$                | Mass $m$, $G_{drive}$, $r_{wheel}$, $k_T$, $I_{slip}$, $R_{bat}$, $a_{fric}$                                   |
| **4. Cornering Lateral Limit**   |   $a_{lat}$    |  m/s²  | $0.85 \cdot a_{fric}$                                                                                                                                         | On-robot tire-carpet traction limit $a_{fric}$                                                                 |
| **5. Max Angular Velocity**      | $\omega_{max}$ | rad/s  | $\eta_{\omega} \dfrac{v_{max}}{R_{max}}$                                                                                                                      | $v_{max}$, outer module lever arm $R_{max} = \max_i \sqrt{x_i^2 + y_i^2}$                                      |
| **6. Max Angular Accel**         | $\alpha_{max}$ | rad/s² | $\eta_{\alpha} \dfrac{m \cdot \min(a_{0,raw}, a_{fric}) \cdot R_{max}}{J_{zz}}$                                                                               | Mass $m$, CAD yaw MOI $J_{zz}$, $R_{max}$, $a_{0,raw}$, $a_{fric}$                                             |
| **7. Cross-Track Time Constant** |  $\tau_{xt}$   |   s    | $\max\left(0.12\text{ s},\; 4 \left(\tau_{lag} + \dfrac{\Delta t_{nom}}{2}\right)\right)$ (lower floor only; preserves $\zeta \ge 1.0$; validate damping)     | Velocity tracking lag $\tau_{lag}$, nominal loop period $\Delta t_{nom}$                                       |
| **8. Max Cross-Track Speed**     | $v_{xt, max}$  |  m/s   | $\min\left(\dfrac{e_{relatch}}{\tau_{xt}}, \; 0.30 \cdot v_{max}\right)$                                                                                      | Configured tracking threshold $e_{relatch}$ ([01](01-path-geometry-and-kinematics.md)), $\tau_{xt}$, $v_{max}$ |
| **9. Heading Feedback Gain**     | $kP_{\theta}$  |  s⁻¹   | $\min\left(8.0\text{ s}^{-1},\; \dfrac{1}{3 \left(\tau_{lag} + \Delta t_{nom} / 2\right)}\right)$                                                               | Velocity tracking lag $\tau_{lag}$, nominal loop period $\Delta t_{nom}$                                       |
| **10. Effective Path-Turn Rate** | $\dot{\delta}_{eff}$ | rad/s | $\min\left(\eta_{steer} \dfrac{V_{nom} - k_{S, steer}}{k_{V, steer} \cdot G_{steer}}, \; \dfrac{\phi_{max}}{\tau_{lag} + \Delta t_{nom} / 2}\right)$ ($\eta_{steer}=0.50, \phi_{max}=0.50\text{ rad}$) | `steerMotor` ($k_{V, steer}$), $k_{S, steer}$, $G_{steer}$, $\tau_{lag}$, $\Delta t_{nom}$ |

### 2.2 Category B: On-Robot Tuned Parameters (3 Constants)

Tune these three primary empirical constants on the robot:

| Parameter                        |    Symbol    | Units | Physical Meaning                                        | Why It Is Tuned on the Robot                                         |
| :------------------------------- | :----------: | :---: | :------------------------------------------------------ | :------------------------------------------------------------------- |
| **1. Friction / Traction Limit** |  $a_{fric}$  | m/s²  | Peak isotropic carpet traction ($\mu \cdot g$)          | Depends on wheel tread compound, wear, and carpet nap                |
| **2. Braking Deceleration**      | $a_{brake}$  | m/s²  | Max safe deceleration without wheel lockup or pitch tip | Depends on CG height $h_{cg}$ pitch weight transfer and regen limits |
| **3. Drivetrain Response Lag**   | $\tau_{lag}$ |   s   | Effective first-order velocity tracking latency         | Aggregates CAN delay, velocity filter window, and steer align lag    |

Setting $(a_{fric}, a_{brake}, \tau_{lag})$ updates ($a_{lat}, a_0, \alpha_{max}, \tau_{xt}, v_{xt, max}, kP_\theta$) by formula. Tracking gains remain controller policy, not physical capabilities; validate these estimates and combined limits on carpet.

---

## 3. Physics Sandbox & Model Validation

Historical sandbox results lack linked runner/configuration; reproduce them before using them as acceptance evidence. They do not establish controller optimality.

Reported historical 2026 testbed setup: **1000 Hz sub-stepped 4-module swerve sandbox** modeled on `CompTunerConstants`:

- **Plant Physics ($1\text{ ms}$ integration, $50\text{ Hz}$ planner):** Mass $m = 61.2\text{ kg}$, yaw MOI $J_{zz} = 5.20\text{ kg}\cdot\text{m}^2$, module lever arms $(\pm 11\text{"}, \pm 11.75\text{"}) \implies R_{chassis} = 0.4088\text{ m}$, $G_{drive} = 7.6737$, $r_{wheel} = 2.0\text{"}$, Kraken X60 FOC torque-speed curves with stator current clamping ($80\text{ A}$) and battery internal resistance voltage sag ($R_{bat} = 0.018\text{ }\Omega$), module steering lag ($\tau_{steer} = 18\text{ ms}$), velocity loop lag ($\tau_{drive} = 25\text{ ms}$), cosine steering compensation, $20\text{ ms}$ pose latency, and per-module isotropic tire-carpet friction circle ($\mu = 1.05$).

- **Test Scenarios:** A 9-waypoint obstacle course combining wide high-speed sweeps, a tight $0.8\text{ m}$ S-chicane, a $160^\circ$ CW whip into a hard heading gate, an intermediate $5\text{ cm}$ stop goal, a near-intersecting loop return, and a final $4\text{ cm} / 2^\circ$ dock (tested both nominally and under a $1.44\text{ m/s}$ lateral collision impulse), plus a self-intersecting figure-8 and a $0.50\text{ m}$ short-leg U-turn.

### 3.1 Reported Sandbox Findings

1. **Commanded vs. Achieved Velocity (`RMSE = 1.989 m/s` in TB1 $\to$ `0.101 m/s` in Revised TB3):** Trailblazer 1 requested $4.75\text{ m/s}$ on a $4.02\text{ m/s}$ drivetrain with no deceleration limit, saturating duty cycle at $100\%$, with $25.3\times$ peak tire slip and $1905\text{ J}$ of tire scrub work.

   Deriving $v_{free} = 4.02\text{ m/s}$ and $v_{max} = \eta_v v_{free}$ from testbed motor/gearing parameters and coupling rotational wheel speed ($v_{max, eff}, a_{fric, eff}$) cut velocity tracking RMSE by **$20\times$** and tire scrub work by **$19\times$** ($101\text{ J}$).

2. **Startup & Endpoint Traces:** Zero-velocity startup testing exposed the first-order lookahead deadlock ($v_0 = 0 \implies s_{eval} = s \implies v = 0$), solved by the second-order lookahead term $\frac{1}{2}a_{avail}(s)\tau_{eff}^2$.

   Stop traces showed $1.3\text{ cm}$ of uncorrected inline lag when $s$ ratcheted to $S_{horizon}$, solved by the terminal 2D Euclidean convergence blend ($< 0.5\text{ mm}$ final error).

3. **Consecutive Corner & Figure-8 Traces:** Re-anchoring leg origins to $T_{out, k-1}$ shrank incoming leg lengths and caused corner $k$ to over-cut `within` ($0.376\text{ m} > 0.35\text{ m}$).

   Preserving vertices with a fixed-origin horizon, decoupling tangent phase-lead $s_{eval, t}$, enforcing coupled symmetric tangent scaling ($G^1$ continuity to $10^{-17}\text{ rad}$), and windowing projection around $s_{high\_water}$ achieved **100% tolerance compliance** across all waypoints and collision perturbations.

| Metric (9-Waypoint Obstacle Course)  |          Legacy TB1          |       TB3 (Uncoupled / Pre-Fix)       |    **TB3 (Revised Derived)**     | **TB3 (Revised + $1.44\text{ m/s}$ Shove)** |
| :----------------------------------- | :--------------------------: | :-----------------------------------: | :------------------------------: | :-----------------------------------------: |
| **Waypoint & Stop Tolerances**       |             FAIL             | FAIL (`0.376m > 0.35m`, `3.92° > 2°`) |          **100% PASS**           |                **100% PASS**                |
| **Velocity Tracking RMSE**           |      $1.989\text{ m/s}$      |          $0.258\text{ m/s}$           |      **$0.101\text{ m/s}$**      |           **$0.114\text{ m/s}$**            |
| **Peak Tire Slip Ratio**             |        $25.33\times$         |             $1.89\times$              |         **$1.11\times$**         |              **$1.18\times$**               |
| **Tire Scrub Work (Energy Lost)**    |      $1905.3\text{ J}$       |           $139.0\text{ J}$            |       **$101.0\text{ J}$**       |            **$152.9\text{ J}$**             |
| **Final Stop Error (Pos / Heading)** | $1.8\text{ cm} / 2.13^\circ$ |     $1.3\text{ cm} / 3.92^\circ$      | **$0.0\text{ cm} / 1.86^\circ$** |      **$0.0\text{ cm} / 1.86^\circ$**       |

---

## 4. Configuration Specification: Java

Store physical inputs/calibration in robot-owned Java records (`RobotDrivetrainSpec` + `EmpiricalDriveCalibration`); derive `DrivetrainLimits` with pure `DrivetrainLimitsCalculator` functions. Keep tracking policy separate ([02](02-velocity-profiler-and-limits.md)); do not duplicate derived values in deploy JSON.

```java
// Proposed input models; values come from the target robot's CAD, motor datasheets, and calibration.
public record RobotDrivetrainSpec(
    double massKg,
    double yawMoiKgMetersSq,
    double wheelRadiusMeters,
    double driveGearRatio,
    double steerGearRatio,
    Translation2d[] modulePositionsMeters,
    DCMotor driveMotor,
    DCMotor steerMotor,
    double slipCurrentAmps,
    double staticFrictionVolts
) {}

public record EmpiricalDriveCalibration(
    double frictionLimit,
    double maxBrakingDeceleration,
    double driveResponseLagSeconds
) {}
```

Validate finite, positive, internally consistent values and enforce startup geometry/calibration guardrails for the target 2027 robot (and any optional carpet testbed):

- Unit & CAD geometry bounds: `0.035 m <= r_wheel <= 0.080 m` (catches inches-vs-meters errors), `20 kg <= massKg <= 80 kg`, `0.28 m <= R_max <= 0.60 m`, module offsets `|x_i|, |y_i| >= 0.18 m`, and `0 < v_max <= v_free`.
- Lag & feedforward-mode consistency: `0.010 s <= tau_lag <= 0.10 s`; recalibrate $\tau_{lag}$ empirically under the **exact downstream feedforward mode** (`ChassisSpeeds`-only vs. `ChassisAccelerations` / `WheelForces` enabled) used on the robot ([02 §5](02-velocity-profiler-and-limits.md#5-cross-track-control-stop-convergence-7-set-2d-projection--feedforwards)).

Record the robot, test conditions, and source of each calibration; testbed or prior comp-bot measurements are not defaults for the new 2027 robot.
