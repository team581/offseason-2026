# Power Management

## Goal

- Be able to last whole match without browning out or experiencing RPM dippage
- Prioritize shooting fast
  - (AKA don’t lower BPS if there’s other things that will save battery)
- Still be able to drive fast in auto/against a defender if absolutely necessary

## Power Manager

- State machine
- Runs on separate thread
- Configures stator current limits to all subsystems on state change
- States
  - Idle (prioritizes swerve/intaking to last whole match)
  - Scoring (prioritizes scoring fast without dropping RPM)
  - Scoring far (prioritizes shooter)
  - Feeding (prioritizes fast throughput)
  - Prioritize intake (prioritizes intaking)
  - Turbo Mode (prioritizes swerve)
  - First auto segment (prioritizes swerve/intake)
