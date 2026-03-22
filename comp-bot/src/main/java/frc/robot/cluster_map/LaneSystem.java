package frc.robot.cluster_map;

import edu.wpi.first.math.geometry.Translation2d;

public class LaneSystem {
  private final double minX, maxX;
  private final double zoneMinY, zoneMaxY;
  private final int numLanes;
  private final double trenchMaxY;

  public LaneSystem(
      double minX, double maxX, double zoneMinY, double zoneMaxY, int numLanes, double trenchMaxY) {
    this.minX = minX;
    this.maxX = maxX;
    this.zoneMinY = zoneMinY;
    this.zoneMaxY = zoneMaxY;
    this.numLanes = numLanes;
    this.trenchMaxY = trenchMaxY;
  }

  public Lane getLaneFromTranslation(Translation2d target) {
    double x = target.getX();
    double y = target.getY();

    if (x >= minX && x <= maxX && y >= zoneMaxY && y <= trenchMaxY) {
      return Lane.SIDE_LANE;
    }

    if (y >= zoneMinY && y < zoneMaxY && x >= minX && x <= maxX) {
      double laneWidth = (maxX - minX) / numLanes;

      // Find which lane index the X coordinate falls into
      int laneIndex = Math.min((int) ((x - minX) / laneWidth), numLanes - 1);

      return switch (laneIndex) {
        case 0 -> Lane.LANE_0;
        // Furthest
        case 1 -> Lane.LANE_1;
        // Middle
        case 2 -> Lane.LANE_2;
        // Closest
        case 3 -> Lane.LANE_3;
        case 4 -> Lane.LANE_4;
        default -> Lane.NONE;
      };
    }
    return Lane.NONE;
  }
}
