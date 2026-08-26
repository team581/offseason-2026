package frc.robot.shooter.feeder;

public enum FeederState {
  SHOOTING(), // Shooting positive values can be assigned later
  BALL_FILLING(), // Ball filling positive values can be assigned later
  INTAKING(), // Intaking negative value can be assigned later
  EJECT(), // Eject negative value
  IDLE(); // Idle remains 0
}
