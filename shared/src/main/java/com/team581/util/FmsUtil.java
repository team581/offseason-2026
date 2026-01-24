package com.team581.util;

import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Range;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;
import java.util.Optional;

public class FmsUtil {
  // seconds
  public static final int TELEOP_TIME = 160;
  public static final int AUTO_TIME = 20;

  // Match period shift times in seconds
  private static final Range<Integer> AUTONOMOUS = Range.closedOpen(0, 20);
  private static final Range<Integer> TRANSITION_SHIFT = Range.closedOpen(20, 30);
  private static final Range<Integer> SHIFT_1 = Range.closedOpen(30, 55);
  private static final Range<Integer> SHIFT_2 = Range.closedOpen(55, 80);
  private static final Range<Integer> SHIFT_3 = Range.closedOpen(80, 105);
  private static final Range<Integer> SHIFT_4 = Range.closedOpen(105, 130);
  private static final Range<Integer> END_GAME = Range.closedOpen(130, 160);
  private static final ImmutableMap<Range<Integer>, Boolean> ALLIANCE_WIN_HUB_ACTIVITY =
      ImmutableMap.ofEntries(
          Map.entry(AUTONOMOUS, true),
          Map.entry(TRANSITION_SHIFT, true),
          Map.entry(SHIFT_1, false),
          Map.entry(SHIFT_2, true),
          Map.entry(SHIFT_3, false),
          Map.entry(SHIFT_4, true),
          Map.entry(END_GAME, true));

  private static final ImmutableMap<Range<Integer>, Boolean> ALLIANCE_LOSS_HUB_ACTIVITY =
      ImmutableMap.ofEntries(
          Map.entry(AUTONOMOUS, true),
          Map.entry(TRANSITION_SHIFT, true),
          Map.entry(SHIFT_1, true),
          Map.entry(SHIFT_2, false),
          Map.entry(SHIFT_3, true),
          Map.entry(SHIFT_4, false),
          Map.entry(END_GAME, true));

  private static final ImmutableSet<Range<Integer>> MATCH_PERIODS =
      ALLIANCE_LOSS_HUB_ACTIVITY.keySet();

  /** Returns time since start of the match, in autonomous. */
  public static int getTimeSinceMatchStart() {
    var time = (int)DriverStation.getMatchTime();
    if (DriverStation.isAutonomous()) {
      return AUTO_TIME - time;
    } else {
      return AUTO_TIME + TELEOP_TIME - time;
    }
  }

  public static Optional<Boolean> isAutoWinner() {
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

  public static boolean isHubActive(int timeSinceMatchStart) {
    var isAutoWinner = isAutoWinner();
    if (isAutoWinner.isEmpty()) {
      return true;
    }
    var possibleRange =
        MATCH_PERIODS.stream().findFirst().filter((a) -> a.contains(timeSinceMatchStart));
    if (possibleRange.isEmpty()) {
      return false;
    }
    return (isAutoWinner.orElseThrow() ? ALLIANCE_WIN_HUB_ACTIVITY : ALLIANCE_LOSS_HUB_ACTIVITY)
        .get(possibleRange.orElseThrow());
  }

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    return alliance == Alliance.Red;
  }

  private FmsUtil() {}
}
