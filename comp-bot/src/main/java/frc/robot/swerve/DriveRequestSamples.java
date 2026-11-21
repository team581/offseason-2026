package frc.robot.swerve;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** One Phoenix writer and one robot-loop reader. Publish only after every column is filled. */
public final class DriveRequestSamples {
  public static final String COLUMNS =
      "phoenix_s,sequence,field_vx_mps,field_vy_mps,omega_radps,field_ax_mps2,field_ay_mps2,"
          + "alpha_radps2,translation_endpoint_shift_s,heading_rad,apply_status,"
          + "robot_fx_N_FL,robot_fx_N_FR,robot_fx_N_BL,robot_fx_N_BR,"
          + "robot_fy_N_FL,robot_fy_N_FR,robot_fy_N_BL,robot_fy_N_BR,yaw_endpoint_shift_s";

  private static final int WIDTH = 20;

  private final String runtimeName;

  private final double[][] rows = new double[128][WIDTH];
  private volatile long written;
  private volatile long read;
  private volatile long dropped;

  DriveRequestSamples(String runtimeName) {
    this.runtimeName = runtimeName;
  }

  public double[] drain() {
    long end = written;
    int count = (int) (end - read);
    double[] batch = new double[count * WIDTH];
    for (int i = 0; i < count; i++) {
      System.arraycopy(rows[(int) ((read + i) % rows.length)], 0, batch, i * WIDTH, WIDTH);
    }
    read = end;
    return batch;
  }

  public long dropped() {
    return dropped;
  }

  public String runtimeName() {
    return runtimeName;
  }

  void capture(
      double time,
      long sequence,
      ChassisSpeeds v,
      ChassisSpeeds a,
      double delay,
      double angularDelay,
      double heading,
      int status,
      Feedforwards forces) {
    long index = written;
    if (index - read >= rows.length) {
      dropped++;
      return;
    }
    double[] row = rows[(int) (index % rows.length)];
    row[0] = time;
    row[1] = sequence;
    row[2] = v.vxMetersPerSecond;
    row[3] = v.vyMetersPerSecond;
    row[4] = v.omegaRadiansPerSecond;
    row[5] = a.vxMetersPerSecond;
    row[6] = a.vyMetersPerSecond;
    row[7] = a.omegaRadiansPerSecond;
    row[8] = delay;
    row[9] = heading;
    row[10] = status;
    System.arraycopy(forces.x_newtons, 0, row, 11, 4);
    System.arraycopy(forces.y_newtons, 0, row, 15, 4);
    row[19] = angularDelay;
    written = index + 1;
  }
}
