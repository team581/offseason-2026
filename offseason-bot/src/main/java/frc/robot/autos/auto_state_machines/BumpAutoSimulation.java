package frc.robot.autos.auto_state_machines;

import com.team581.util.FieldUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.imu.Imu;

/** Simple ramp geometry for the bump autos' desktop simulation, not a chassis dynamics model. */
final class BumpAutoSimulation {
  static void update(Imu imu, Pose2d pose) {
    double fieldTilt = 0.0;
    for (var center :
        new Pose2d[] {
          FieldUtil.RED_DEPOT_BUMP_CENTER,
          FieldUtil.RED_OUTPOST_BUMP_CENTER,
          FieldUtil.pathflip(FieldUtil.RED_DEPOT_BUMP_CENTER),
          FieldUtil.pathflip(FieldUtil.RED_OUTPOST_BUMP_CENTER)
        }) {
      double dx = pose.getX() - center.getX();
      if (MathUtil.isNear(pose.getY(), center.getY(), 0.9)
          && MathUtil.isNear(0, dx, 0.56)
          && !MathUtil.isNear(0, dx, 0.04)) {
        fieldTilt = Math.copySign(15.0, dx);
      }
    }
    imu.setPitch(fieldTilt * pose.getRotation().getCos());
    imu.setRoll(-fieldTilt * pose.getRotation().getSin());
  }

  private BumpAutoSimulation() {}
}
