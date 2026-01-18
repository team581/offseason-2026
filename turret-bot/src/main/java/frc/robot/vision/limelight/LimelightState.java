package frc.robot.vision.limelight;

public enum LimelightState {
  OFF(1),
  TAGS(1),
  HUB_TAGS(1),
  HANDOFF(5);

  final int pipelineIndex;

  LimelightState(int index) {
    this.pipelineIndex = index;
  }
}
