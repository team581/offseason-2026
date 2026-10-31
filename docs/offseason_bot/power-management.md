# Power Management

## Goal

- Last the whole match without browning out or dropping flywheel RPM
- Prioritize shooting fast (avoid lowering BPS if other mechanisms can save power instead)
- Preserve high drive speed in auto or when pushing through defense

## Power Manager

- State machine
- Run on separate thread
- Configure stator current limits across all subsystems on state change
- States:
  - Idle (prioritize swerve/intaking to last whole match)
  - Scoring (prioritize scoring fast without dropping RPM)
  - Scoring far (prioritize shooter)
  - Feeding (prioritize fast throughput)
  - Prioritize intake (prioritize intaking)
  - Turbo Mode (prioritize swerve)
  - First auto segment (prioritize swerve/intake)
