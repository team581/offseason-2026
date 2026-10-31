# Automation

## Scoring

**MUST**

- Aim turret/hood at hub
- Hub state tracking:
  - Begin scoring before hub is active using fuel ToF
- [X swerve](automation.md#x-swerve) if no driver input:
  - Check requested speeds, not current speeds
- Use operator joystick to adjust if needed
- Limit drive speeds

**Fallbacks:**

- Switch to hardcoded shot parameters if vision fails:
  - Read driver station position (1, 2, or 3) from FMS
  - If positioned on the right driver station, use the right trench
  - Otherwise, use the left trench

## Feeding

**MUST**

- Prioritize goal pose closer to tower for harder stealing
  - Switch to field corner if blocked by hub
- Avoid hitting the hub at all times while feeding
- Use operator joystick to adjust away from other robots
- X Swerve if no driver input

**Fallbacks:**

- Require accurate pose estimate to calculate shot parameters
  - If vision fails, assume a hardcoded pose depending on feed state:
    - Robot X coordinate is a known value, with distance effectively fixed
- Disable feeding shot collision detection entirely if vision fails

## X Swerve

**MUST**

- Command X swerve if shooting while requested velocity and heading PID output are low
- Purposefully command slower steer module speeds while applying X swerve to prevent repeatedly breaking aiming tolerance

**Fallbacks:**

- Apply X Swerve from fallback shot locations

## Safe Shooter Hood

**MUST**

- Keep hood stowed down when not scoring or feeding
- Ensure shooter hood avoids hitting the trench when raised:
  - Near trench -> fully stowed
  - Inside alliance zone -> track hub
  - Outside alliance zone -> track feeding

**Fallbacks:**

- Keep hood stowed while idling if vision fails
- Provide operator force button to shoot from under defense
