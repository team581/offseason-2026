package frc.robot.autos.auto_state_machines.auto_states;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.Points;

public enum BumpR6MidlineAutoState {
  INTAKING(Points.INTAKE_MIDLINE_RIGHT),
  SCORE(Points.HUB_CENTER);

  public final Points point;

  private BumpR6MidlineAutoState(Points point) {
    this.point = point;
  }

  public Pose2d getPose() {
    return this.point.getPose();
  }
}
