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
  private final Consumer<Translation2d> poseResetConsumer;
  private Rotation2d crossingDirection = Rotation2d.kZero;
  private double directionalTilt = 0.0;
  private boolean isFlat = true;
  private boolean isFlatFallbackDebounced = false;
  private Point landingPoint = new Point(Pose2d.kZero, Pose2d.kZero);

  public BumpCrossingTracker(
      DoubleSupplier pitchSupplier,
      DoubleSupplier rollSupplier,
      Consumer<Translation2d> poseResetConsumer) {
    super(BumpCrossingState.NOT_ON_BUMP);
    this.poseResetConsumer = poseResetConsumer;
    this.pitchSupplier = pitchSupplier;
    this.rollSupplier = rollSupplier;
  }

  @Override
  protected void collectInputs() {
    // Get the tilt relative to the known crossing direction (set via bumpCrossRequest)
    // Positive tilt should be tilted up toward the crossing direction
    var pitch = pitchSupplier.getAsDouble();
    var roll = rollSupplier.getAsDouble();
    var projection =
        (pitch * Math.cos(crossingDirection.getRadians()))
            + (roll * Math.sin(crossingDirection.getRadians()));
    directionalTilt = Math.signum(projection) * Math.hypot(pitch, roll);
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
      return BumpCrossingState.NOT_ON_BUMP;
    }

    return switch (currentState) {
      case NOT_ON_BUMP -> {
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
          yield BumpCrossingState.NOT_ON_BUMP;
        }
        yield currentState;
      }
    };
  }

  public void bumpCrossRequest(Point landingPoint, Rotation2d crossingDirection) {
    this.landingPoint = landingPoint;
    this.crossingDirection = crossingDirection;
    DogLog.timestamp("Imu/BumpCrossing/CrossRequest");
  }

  @Override
  protected void beforeTransition(BumpCrossingState oldState, BumpCrossingState newState) {
    if (oldState == BumpCrossingState.CROSSING_DOWNHILL
        && newState == BumpCrossingState.NOT_ON_BUMP) {
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
