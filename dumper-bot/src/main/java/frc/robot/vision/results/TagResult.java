package frc.robot.vision.results;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public class TagResult {
  private Pose2d pose;
  private double timestamp;
  private Vector<N3> standardDevs;

  public Pose2d pose() {
    return pose;
  }

  public Vector<N3> standardDevs() {
    return standardDevs;
  }

  public double timestamp() {
    return timestamp;
  }

  void update(Pose2d pose, double timestamp, Vector<N3> standardDevs) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.standardDevs = standardDevs;
  }
}
