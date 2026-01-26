package com.team581.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public class FmsUtil {
  // seconds
  public static final double TELEOP_TIME = 160.0;
  public static final double AUTO_TIME = 20.0;

  // Match period shift times in seconds
  public static final double transitionShiftTimeStamp = 20.0;
  public static final double shift1TimeStamp = 30.0;
  public static final double shift2TimeStamp = 55.0;
  public static final double shift3TimeStamp = 80.0;
  public static final double shift4TimeStamp = 105.0;
  public static final double endGameTimeStamp = 130.0;

  public static Optional<Boolean> isAutoWinner() {
    if (RobotBase.isSimulation()) {
      return Optional.of(true);
    }
    var gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return Optional.empty();
    }
    var character = gameData.charAt(0);
    return switch (character) {
      case 'R' -> Optional.of(isRedAlliance());
      case 'B' -> Optional.of(!isRedAlliance());
      default -> Optional.empty();
    };
  }

  public static boolean isHubActive(double timeSinceMatchStart) {
    var isAutoWinner = isAutoWinner();
    if (isAutoWinner.isEmpty()) {
      return true;
    }
    if (timeSinceMatchStart >= endGameTimeStamp) {
      return true;
    }
    if (timeSinceMatchStart >= shift4TimeStamp) {
      return isAutoWinner.orElseThrow();
    }
    if (timeSinceMatchStart >= shift3TimeStamp) {
      return !isAutoWinner.orElseThrow();
    }
    if (timeSinceMatchStart >= shift2TimeStamp) {
      return isAutoWinner.orElseThrow();
    }
    if (timeSinceMatchStart >= shift1TimeStamp) {
      return !isAutoWinner.orElseThrow();
    }
    if (timeSinceMatchStart >= transitionShiftTimeStamp) {
      return true;
    }
    return true;
  }

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    return alliance == Alliance.Red;
  }

  private FmsUtil() {}
}
