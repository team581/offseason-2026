# Automation

## Scoring

**MUST**

- Aim turret/hood at hub
- Hub state tracking tech
  - Begins scoring before hub is active, use fuel ToF
- [X swerve](automation.md#x-swerve) if no driver input
  - Need to check requested speeds, not current speeds
- Use operator joystick to adjust if needed
- Limit drive speeds

**Fallbacks:**

- Switch to hardcoded shot parameters if vision fails
  - Read our driver station position (1, 2, or 3) from FMS
  - If we are on the right driver station, use the right trench
  - Otherwise, use the left trench

## Feeding

**MUST**

- Prioritizes goal pose closer to tower for harder stealing
  - Switches to field corner if blocked by hub
- Make it so that when you’re feeding, we don’t hit hub ever
- Use operator joystick to adjust away from other robots
- X Swerve if no driver input

**Fallbacks:**

- Requires accurate pose estimate to calculate shot parameters
  - If we lose vision, should assume a hardcoded pose depending on feed state
    - Robot X coordinate will be a known value, but Y coordinate could be anything
    - This means the distance is effectively statically known
- Feeding shot collision detection should be disabled entirely if we lose vision

## X Swerve

**MUST**

- If we are shooting while the requested velocity and snaps PID output are low, X the swerve
- Purposefully command slower steer module speeds while applying the X so that the robot does not repeatedly move out of aiming tolerance

**Fallbacks:**

- Still X Swerve, just from the fallback shot location

## Safe Shooter Hood

**MUST**

- If we aren’t scoring/feeding, hood always down
- When shooter hood is up (scoring, feeding), we should make sure that it won’t hit the trench
  - Near trench -> fully stowed
  - Inside alliance zone -> tracks hub
  - Not inside alliance zone -> tracks feeding

**Fallbacks:**

- Needs accurate pose estimate, if vision dies the hood will always be stowed while idling anyway
- If under defense and that’s the only place to score, operator has force button
