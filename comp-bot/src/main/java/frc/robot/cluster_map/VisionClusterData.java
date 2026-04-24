package frc.robot.cluster_map;

import edu.wpi.first.math.geometry.Translation2d;

class VisionClusterData {
  private Translation2d translation = Translation2d.kZero;
  private int size = 0;
  private double score = 0.0;

  double score() {
    return score;
  }

  int size() {
    return size;
  }

  Translation2d translation() {
    return translation;
  }

  void update(Translation2d translation, int size, double score) {
    this.translation = translation;
    this.size = size;
    this.score = score;
  }
}
