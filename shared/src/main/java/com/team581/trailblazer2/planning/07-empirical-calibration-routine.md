# Trailblazer 2 Design Specification: Drivetrain Parameter Derivation & Physics Sandbox Validation

**Document ID:** TB2-SPEC-07  
**Status:** Approved for Implementation  
**Author:** Team 581 Technical Staff  
**Applies To:** `DrivetrainLimits`, `TunerConstants`, `drivetrain-limits.json`  

---

## 1. Overview & Principles

A major source of fragile auto tuning in FRC is embedding assumed physical parameters (e.g. `4.5 m/s` velocity, `8.0 m/s^2` acceleration) and per-waypoint PID gains directly into individual path definitions.

Trailblazer 2 enforces a strict physical boundary:
- **Autos declare operational intent and geometric tolerances** (e.g. `within(0.25)`, `limits: fast` $= 0.90 \times \text{limits}$, `limits: careful` $= 0.60 \times \text{limits}$).
- **Physical capabilities belong to the drivetrain**, stored in `drivetrain-limits.json`. Nine of the twelve parameters are mathematically derived from CAD and motor datasheets before the robot ever touches carpet, leaving only three physical parameters ($a_{fric}, a_{brake}, \tau_{lag}$) to tune on the robot during bringup.

---

## 2. Mathematically Derived vs. On-Robot Tuned Parameters

To enable a **$\le 1$ day robot bringup** where path following works out of the box without per-auto PID tuning, Trailblazer 2 separates parameters into **Mathematically Derived Constants** (computed from CAD, gearing, and Kraken X60 FOC motor datasheets) and **On-Robot Tuned Constants** (3 carpet/latency properties tuned once on the physical robot).

### 2.1 Category A: Mathematically Derived Parameters (9 Constants from CAD & Datasheet)

Given CAD geometry, `TunerConstants` ($G_{drive}$, $r_{wheel}$, $L_x, L_y \implies R_{chassis} = \sqrt{L_x^2 + L_y^2}$, mass $m$, yaw MOI $J_{zz}$, stator limit $I_{slip}$, battery resistance $R_{bat} \approx 0.018\text{ }\Omega$, static friction $k_S \approx 0.20\text{ V}$), and Kraken X60 FOC datasheet constants ($V_{nom} = 12\text{ V}$, $\omega_{free} = 607.37\text{ rad/s}$, $\tau_{stall} = 9.37\text{ N}\cdot\text{m}$, $I_{stall} = 483\text{ A}$, $I_{free} = 2.0\text{ A}$, $R_m = 12/I_{stall} = 0.0248\text{ }\Omega$, $k_T = \tau_{stall}/I_{stall} = 0.0194\text{ N}\cdot\text{m/A}$, $k_V = (12 - I_{free}R_m)/\omega_{free} = 0.01967\text{ V/(rad/s)}$, peak current $I_{max} = \min(I_{slip}, (V_{oc} - k_S)/(R_m + 4 R_{bat}))$, $\eta_{gear} \approx 0.90$):

| Parameter | Symbol | Units | Mathematical Derivation Formula | Required Input Constraints |
| :--- | :---: | :---: | :--- | :--- |
| **1. Motor Free Speed** | $v_{free}$ | m/s | $\dfrac{V_{nom} - k_S}{k_V \cdot G_{drive}} \cdot r_{wheel}$ | $G_{drive}$, $r_{wheel}$, motor $k_V$, $k_S$ |
| **2. Loaded Top Speed** | $v_{max}$ | m/s | $\eta_v \cdot v_{free}$ ($\eta_v \approx 0.90\text{–}0.92$) | $v_{free}$, battery sag & steering cosine headroom |
| **3. Stall Acceleration** | $a_0$ | m/s² | $\eta_a \min\left(\dfrac{4 \eta_{gear} G_{drive} k_T (I_{max} - I_{free})}{m \cdot r_{wheel}}, \; a_{fric}\right)$ | Mass $m$, $G_{drive}$, $r_{wheel}$, $k_T$, $I_{slip}$, $R_{bat}$, $a_{fric}$ |
| **4. Cornering Lateral Limit** | $a_{lat}$ | m/s² | $0.85 \cdot a_{fric}$ | On-robot tire-carpet traction limit $a_{fric}$ |
| **5. Max Angular Velocity** | $\omega_{max}$ | rad/s | $\eta_{\omega} \dfrac{v_{max}}{R_{chassis}}$ | $v_{max}$, module lever arm $R_{chassis} = \sqrt{L_x^2 + L_y^2}$ |
| **6. Max Angular Accel** | $\alpha_{max}$ | rad/s² | $\min\left(\eta_{\alpha} \dfrac{m \cdot a_{fric} \cdot R_{chassis}}{J_{zz}}, \; \dfrac{\beta_{rot} \cdot a_{fric}}{R_{chassis}}\right)$ | Mass $m$, CAD yaw MOI $J_{zz}$, $R_{chassis}$, $a_{fric}$ ($\beta_{rot} \in [0.35, 0.50]$) |
| **7. Cross-Track Time Constant** | $\tau_{xt}$ | s | $4 \left(\tau_{lag} + \dfrac{\Delta t}{2}\right)$ (Critical damping $\zeta \ge 1$) | Velocity tracking lag $\tau_{lag}$, loop period $\Delta t$ |
| **8. Max Cross-Track Speed** | $v_{xt, max}$ | m/s | $\min\left(\dfrac{e_{relatch}}{\tau_{xt}}, \; 0.30 \cdot v_{max}\right)$ | Re-latch threshold $e_{relatch} = 0.40\text{ m}$, $\tau_{xt}$, $v_{max}$ |
| **9. Heading Feedback Gain** | $kP_{\theta}$ | s⁻¹ | $\dfrac{1}{3 \left(\tau_{lag} + \Delta t / 2\right)}$ | Velocity tracking lag $\tau_{lag}$, loop period $\Delta t$ |

*(Note: The low-level coaxial module azimuth-drive mechanical coupling ratio $k_{couple} = N_{ring} / N_{pinion}$ is also derived directly from CAD gear tooth counts in `TunerConstants`).*

### 2.2 Category B: On-Robot Tuned Parameters (3 Constants)

Only **three** physical constants depend on venue carpet grip, CG pitch weight transfer, and closed-loop actuator latency, and are tuned on the physical robot:

| Parameter | Symbol | Units | Physical Meaning | Why It Is Tuned on the Robot |
| :--- | :---: | :---: | :--- | :--- |
| **1. Friction / Traction Limit** | $a_{fric}$ | m/s² | Peak isotropic carpet traction ($\mu \cdot g$) | Depends on wheel tread compound, wear, and carpet nap |
| **2. Braking Deceleration** | $a_{brake}$ | m/s² | Max safe deceleration without wheel lockup or pitch tip | Depends on CG height $h_{cg}$ pitch weight transfer and regen limits |
| **3. Drivetrain Response Lag** | $\tau_{lag}$ | s | Effective first-order velocity tracking latency | Aggregates CAN delay, velocity filter window, and steer align lag |

Once $(a_{fric}, a_{brake}, \tau_{lag})$ are set on the robot, the dependent derived parameters ($a_{lat}, a_0, \alpha_{max}, \tau_{xt}, v_{xt, max}, kP_\theta$) update automatically by formula.

---

## 3. Physics Sandbox & Objective Model Validation

To objectively evaluate the mathematical model before hardware bringup, a **1000 Hz sub-stepped 4-module swerve physics sandbox** was constructed using Team 581's physical constants (`CompTunerConstants`):
- **Plant Physics ($1\text{ ms}$ integration, $50\text{ Hz}$ planner):** Mass $m = 61.2\text{ kg}$, yaw MOI $J_{zz} = 5.20\text{ kg}\cdot\text{m}^2$, module lever arms $(\pm 11\text{"}, \pm 11.75\text{"}) \implies R_{chassis} = 0.4088\text{ m}$, $G_{drive} = 7.6737$, $r_{wheel} = 2.0\text{"}$, Kraken X60 FOC torque-speed curves with stator current clamping ($80\text{ A}$) and battery internal resistance voltage sag ($R_{bat} = 0.018\text{ }\Omega$), module steering lag ($\tau_{steer} = 18\text{ ms}$), velocity loop lag ($\tau_{drive} = 25\text{ ms}$), cosine steering compensation, $20\text{ ms}$ pose latency, and per-module isotropic tire-carpet friction circle ($\mu = 1.05$).
- **Test Scenarios:** A 9-waypoint obstacle course combining wide high-speed sweeps, a tight $0.8\text{ m}$ S-chicane, a $160^\circ$ CW whip into a hard heading gate, an intermediate $5\text{ cm}$ stop goal, a near-intersecting loop return, and a final $4\text{ cm} / 2^\circ$ dock (tested both nominally and under a $1.44\text{ m/s}$ lateral collision impulse), plus a self-intersecting figure-8 and a $0.50\text{ m}$ short-leg U-turn.

### 3.1 How Sandbox Data Identified the Objective Optimal Model
Comparing telemetry across model variants directly isolated the mathematical failure modes that pure kinematic analysis missed:
1. **Commanded vs. Achieved Velocity (`RMSE = 1.989 m/s` in TB1 $\to$ `0.101 m/s` in Optimal TB2):** Trailblazer 1 requested $4.75\text{ m/s}$ on a $4.02\text{ m/s}$ drivetrain with no deceleration limit, saturating motor duty cycle at $100\%$, causing $25.3\times$ peak tire slip and $1905\text{ J}$ of tire scrub work. Deriving $v_{free} = 4.02\text{ m/s}$ and $v_{max} = \eta_v v_{free}$ from `TunerConstants` and coupling rotational wheel speed ($v_{max, eff}, a_{fric, eff}$) cut velocity tracking RMSE by **$20\times$** and tire scrub work by **$19\times$** ($101\text{ J}$).
2. **Startup & Endpoint Traces:** Zero-velocity startup testing exposed the first-order lookahead deadlock ($v_0 = 0 \implies s_{eval} = s \implies v = 0$), solved by the second-order lookahead term $\frac{1}{2}a_{avail}(s)\tau_{eff}^2$. Stop-goal residual traces showed $1.3\text{ cm}$ of uncorrected inline lag when $s$ ratcheted to $S_{horizon}$, solved by the terminal 2D Euclidean convergence blend ($< 0.5\text{ mm}$ final error).
3. **Consecutive Corner & Figure-8 Traces:** Re-anchoring leg origins to $T_{out, k-1}$ shrank incoming leg lengths and caused corner $k$ to over-cut `within` ($0.376\text{ m} > 0.35\text{ m}$). Preserving waypoint vertices via sub-segment latching, decoupling tangent phase-lead $s_{eval, t}$, enforcing coupled symmetric tangent scaling ($G^1$ continuity to $10^{-17}\text{ rad}$), and windowing projection around $s_{high\_water}$ achieved **100% tolerance compliance** across all waypoints and collision perturbations.

| Metric (9-Waypoint Obstacle Course) | Legacy TB1 | TB2 (Uncoupled / Pre-Fix) | **TB2 (Optimal Derived)** | **TB2 (Optimal + $1.44\text{ m/s}$ Shove)** |
| :--- | :---: | :---: | :---: | :---: |
| **Waypoint & Stop Tolerances** | FAIL | FAIL (`0.376m > 0.35m`, `3.92° > 2°`) | **100% PASS** | **100% PASS** |
| **Velocity Tracking RMSE** | $1.989\text{ m/s}$ | $0.258\text{ m/s}$ | **$0.101\text{ m/s}$** | **$0.114\text{ m/s}$** |
| **Peak Tire Slip Ratio** | $25.33\times$ | $1.89\times$ | **$1.11\times$** | **$1.18\times$** |
| **Tire Scrub Work (Energy Lost)** | $1905.3\text{ J}$ | $139.0\text{ J}$ | **$101.0\text{ J}$** | **$152.9\text{ J}$** |
| **Final Stop Error (Pos / Heading)** | $1.8\text{ cm} / 2.13^\circ$ | $1.3\text{ cm} / 3.92^\circ$ | **$0.0\text{ cm} / 1.86^\circ$** | **$0.0\text{ cm} / 1.86^\circ$** |

---

## 4. Configuration Specification: `drivetrain-limits.json`

The 12 drivetrain parameters (9 mathematically derived + 3 tuned on robot) are stored in `src/main/deploy/drivetrain-limits.json`:

```json
{
  "schema_version": 2,
  "robot": "comp-bot",
  "limits": {
    "max_linear_velocity": 3.85,
    "free_linear_acceleration": 6.20,
    "motor_free_speed": 4.02,
    "max_braking_deceleration": 4.10,
    "friction_limit": 3.85,
    "max_lateral_acceleration": 3.27,
    "max_angular_velocity": 8.40,
    "max_angular_acceleration": 26.00,
    "drive_response_lag_seconds": 0.040,
    "cross_track_time_constant": 0.200,
    "max_cross_track_correction": 1.200,
    "heading_kp": 6.670
  }
}
```

### 4.1 Git Version Control & CI Replay Gate
Whenever `drivetrain-limits.json` is updated, GitHub Actions CI automatically replays the suite of competition autos against the limits in simulation to verify that all autos complete within their 15.0-second match budgets and satisfy all stop tolerances and keepouts.
