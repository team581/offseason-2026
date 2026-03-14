package com.team581.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public class FmsUtil {
  // Match period shift times in seconds
  private static final double SHIFT1_TIMESTAMP = 30.0;
  private static final double SHIFT2_TIMESTAMP = 55.0;
  private static final double SHIFT3_TIMESTAMP = 80.0;
  private static final double SHIFT4_TIMESTAMP = 105.0;
  private static final double ENDGAME_TIMESTAMP = 130.0;

  private static final double ENDGAME_DURATION = 160.0;
  private static final double SHIFT4_TIME_DURATION = 130.0;
  private static final double SHIFT3_TIME_DURATION = 105.0;
  private static final double SHIFT2_TIME_DURATION = 80.0;
  private static final double SHIFT1_TIME_DURATION = 55.0;
  private static final double TRANSITION_DURATION = 30.0;

  public static final double MATCH_TIME_AT_TELEOP_START = 20.0;

  public static String currentShift(double timeSinceMatchStart) {
    if (DriverStation.isDisabled() || !DriverStation.isTeleop()) {
      return "";
    }
    var maybeWonAuto = isAutoWinner();
    if (timeSinceMatchStart <= TRANSITION_DURATION) {
      if (maybeWonAuto.isEmpty()) {
        return "TRANSITION";
      }
      if (maybeWonAuto.get()) {
        return "WON";
      }
      return "LOST";
    }
    if (timeSinceMatchStart <= SHIFT1_TIME_DURATION) {
      return "SHIFT 1";
    }
    if (timeSinceMatchStart <= SHIFT2_TIME_DURATION) {
      return "SHIFT 2";
    }
    if (timeSinceMatchStart <= SHIFT3_TIME_DURATION) {
      return "SHIFT 3";
    }
    if (timeSinceMatchStart <= SHIFT4_TIME_DURATION) {
      return "SHIFT 4";
    }
    if (timeSinceMatchStart <= ENDGAME_DURATION) {
      return "ENDGAME";
    }
    return "";
  }

  public static boolean isHubActive(double timeSinceMatchStart, boolean defaultAutoWinnerValue) {
    var isAutoWinner = isAutoWinner().orElse(defaultAutoWinnerValue);

    if (timeSinceMatchStart >= ENDGAME_TIMESTAMP) {
      return true;
    }
    if (timeSinceMatchStart >= SHIFT4_TIMESTAMP) {
      return isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT3_TIMESTAMP) {
      return !isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT2_TIMESTAMP) {
      return isAutoWinner;
    }
    if (timeSinceMatchStart >= SHIFT1_TIMESTAMP) {
      return !isAutoWinner;
    }
    return true;
  }

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    return alliance == Alliance.Red;
  }

  public static double timeUntilNextShift(double timeSinceMatchStart) {
    if (DriverStation.isDisabled() || !DriverStation.isTeleop()) {
      return 0.0;
    }

    if (timeSinceMatchStart <= TRANSITION_DURATION) {
      return TRANSITION_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT1_TIME_DURATION) {
      return SHIFT1_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT2_TIME_DURATION) {
      return SHIFT2_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT3_TIME_DURATION) {
      return SHIFT3_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= SHIFT4_TIME_DURATION) {
      return SHIFT4_TIME_DURATION - timeSinceMatchStart;
    }
    if (timeSinceMatchStart <= ENDGAME_DURATION) {
      return ENDGAME_DURATION - timeSinceMatchStart;
    }
    return 0.0;
  }

  private static Optional<Boolean> isAutoWinner() {
    if (RobotBase.isSimulation()) {
      return Optional.empty();
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

  private FmsUtil() {}
}
