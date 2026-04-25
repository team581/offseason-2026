package com.team581.mechanisms.imu;

import com.team581.autos.Point;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

public class BumpCrossingTracker extends StateMachine<BumpCrossingState> {
  private static final double FLAT_DEBOUNCE_SECONDS = 0.04;
  private static final double FLAT_FALLBACK_DEBOUNCE_SECONDS = 0.75;
  private static final DoubleSubscriber FLAT_THRESHOLD =
      DogLog.tunable("BumpCrossing/FlatThresholdDegrees", 5.0);
  private static final DoubleSubscriber CROSSING_THRESHOLD =
      DogLog.tunable("BumpCrossing/CrossingThresholdDegrees", 5.0);

  private final Debouncer flatDebouncer =
      new Debouncer(FLAT_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final Debouncer flatFallbackDebouncer =
      new Debouncer(FLAT_FALLBACK_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final DoubleSupplier pitchSupplier;
  private final DoubleSupplier rollSupplier;
  private final DoubleSupplier headingSupplier;
  private final Consumer<Translation2d> poseResetConsumer;
  private Rotation2d crossingDirection = Rotation2d.kZero;
  private double directionalTilt = 0.0;
  private boolean isFlat = true;
  private boolean isFlatFallbackDebounced = false;
  private Point landingPoint = new Point(Pose2d.kZero, Pose2d.kZero);

  public BumpCrossingTracker(
      DoubleSupplier pitchSupplier,
      DoubleSupplier rollSupplier,
      DoubleSupplier headingSupplier,
      Consumer<Translation2d> poseResetConsumer) {
    super(BumpCrossingState.FLAT_NOT_CROSSING);
    this.poseResetConsumer = poseResetConsumer;
    this.pitchSupplier = pitchSupplier;
    this.rollSupplier = rollSupplier;
    this.headingSupplier = headingSupplier;
  }

  /**
   * Calculates the signed tilt component along the crossing direction.
   *
   * <p>Pitch and roll are robot-frame, while crossing direction is field-frame. The robot's heading
   * is used to rotate the tilt gradient from robot frame into field frame. The result is the scalar
   * projection of the field-frame tilt gradient onto the crossing direction: positive when tilted
   * up toward the crossing direction and negative when tilted away. The magnitude reaches
   * hypot(pitch, roll) when the gradient is fully aligned with the crossing direction and varies
   * smoothly to zero (and through to the opposite sign) as the alignment changes.
   */
  public static double calculateDirectionalTilt(
      double pitchDegrees,
      double rollDegrees,
      double headingDegrees,
      Rotation2d crossingDirection) {
    var heading = Math.toRadians(headingDegrees);
    // Rotate the robot-frame tilt gradient (pitch, roll) into field frame by the robot's heading
    var tiltXField = pitchDegrees * Math.cos(heading) - rollDegrees * Math.sin(heading);
    var tiltYField = pitchDegrees * Math.sin(heading) + rollDegrees * Math.cos(heading);
    DogLog.log("Imu/BumpCrossing/TiltXField", tiltXField);
    DogLog.log("Imu/BumpCrossing/TiltYField", tiltYField);
    // Project the field-frame tilt onto the crossing direction
    var tiltXProjection = (tiltYField * Math.cos(crossingDirection.getRadians()));
    var tiltYProjection = (tiltXField * Math.sin(crossingDirection.getRadians()));
    DogLog.log("Imu/BumpCrossing/TiltXProjection", tiltXProjection);
    DogLog.log("Imu/BumpCrossing/TiltYProjection", tiltYProjection);
    return tiltXProjection + tiltYProjection;
  }

  @Override
  protected void collectInputs() {
    // Get the tilt relative to the known crossing direction (set via bumpCrossRequest)
    // Positive tilt should be tilted up toward the crossing direction
    directionalTilt =
        calculateDirectionalTilt(
            pitchSupplier.getAsDouble(),
            rollSupplier.getAsDouble(),
            headingSupplier.getAsDouble(),
            crossingDirection);
    isFlat = flatDebouncer.calculate(Math.abs(directionalTilt) < FLAT_THRESHOLD.get());
    isFlatFallbackDebounced =
        flatFallbackDebouncer.calculate(Math.abs(directionalTilt) < FLAT_THRESHOLD.get());
  }

  @Override
  public BumpCrossingState getNextState(BumpCrossingState currentState) {
    // Fallback
    if (currentState == BumpCrossingState.CROSSING_UPHILL && isFlatFallbackDebounced) {
      poseResetConsumer.accept(landingPoint.getTranslation());
      DogLog.timestamp("Imu/BumpCrossing/FallbackFinishedCrossing");
      return BumpCrossingState.FLAT_NOT_CROSSING;
    }

    return switch (currentState) {
      case FLAT_ABOUT_TO_CROSS -> {
        if (directionalTilt > CROSSING_THRESHOLD.get()) {
          yield BumpCrossingState.CROSSING_UPHILL;
        }
        yield currentState;
      }
      case CROSSING_UPHILL -> {
        if (directionalTilt < -CROSSING_THRESHOLD.get()) {
          yield BumpCrossingState.CROSSING_DOWNHILL;
        }
        yield currentState;
      }
      case CROSSING_DOWNHILL -> {
        if (isFlat) {
          yield BumpCrossingState.FLAT_NOT_CROSSING;
        }
        yield currentState;
      }
      case FLAT_NOT_CROSSING -> currentState;
    };
  }

  public void bumpCrossRequest(Point landingPoint, Rotation2d crossingDirection) {
    this.landingPoint = landingPoint;
    this.crossingDirection = crossingDirection;
    DogLog.timestamp("Imu/BumpCrossing/CrossRequest");
    setStateFromRequest(BumpCrossingState.FLAT_ABOUT_TO_CROSS);
  }

  @Override
  protected void beforeTransition(BumpCrossingState oldState, BumpCrossingState newState) {
    if (oldState == BumpCrossingState.CROSSING_DOWNHILL
        && newState == BumpCrossingState.FLAT_NOT_CROSSING) {
      // We just crossed, reset pose
      poseResetConsumer.accept(landingPoint.getTranslation());
      DogLog.timestamp("Imu/BumpCrossing/FinishedCrossing");
    }
  }

  public void log() {
    DogLog.log("Imu/BumpCrossing/State", getState());
    DogLog.log("Imu/BumpCrossing/CrossingDirection", crossingDirection);
    DogLog.log("Imu/BumpCrossing/DirectionalTilt", directionalTilt);
    DogLog.log("Imu/BumpCrossing/IsFlat", isFlat);
    DogLog.log("Imu/BumpCrossing/IsFlatFallbackDebounced", isFlatFallbackDebounced);
  }
}
