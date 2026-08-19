package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;

public class ShooterConfig {
  public static final TalonFXConfiguration TOP_LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(
                      60)); // assign currents later thru tuning but yes 60 is a reasonable
  // startpoint for now
  public static final TalonFXConfiguration TOP_RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(
                      60)); // assign currents later thru tuning but yes 60 is a reasonable
  // startpoint for now
  public static final TalonFXConfiguration BOTTOM_LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(
                      60)); // assign currents later thru but yes 60 is a reasonable startpoint for
  // now
  public static final TalonFXConfiguration BOTTOM_RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(
                      60)); // assign currents later thru  but yes 60 is a reasonable startpoint for
  // now

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM = // values given later
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreRPM",
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0));

  public static final PolynomialRegression SCORING_REGRESSION_MODEL = // values given later
      PolynomialRegression.quadratic("Shooter/ScoringRegression", DISTANCE_TO_SCORE_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedingRPM",
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0));

  public static final PolynomialRegression FEEDING_REGRESSION_MODEL = // values given later
      PolynomialRegression.quadratic("Shooter/FeedingRegression", DISTANCE_TO_FEEDING_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreToF",
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0));
  public static final PolynomialRegression SCORING_TOF_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringToFRegression", DISTANCE_TO_SCORE_TOF);

  public static final InterpolatingDoubleTreeMap
      DISTANCE_TO_FEED_TOF = // if I want to do SOM will have to watch and review film and research
          // to see how many balls are released through testing
          TunableInterpolatingDoubleTreeMap.ofEntries(
              "Shooter/DistanceToFeedToF",
              Map.entry(0.0, 0.0),
              Map.entry(0.0, 0.0),
              Map.entry(0.0, 0.0));
  public static final PolynomialRegression FEEDING_TOF_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingToFRegression", DISTANCE_TO_FEED_TOF);

  public static final double MAX_SAFE_RPM = 0; // can be adjusted
  public static final double IDLE_RPM = 0;
  public static final double RPM_TOLERANCE_FEEDING = 0;
  public static final double RPM_TOLERANCE_SHOOTING = 0;
  public static double RPM_TOLERANCE;
}
