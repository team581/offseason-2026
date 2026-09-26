package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(50));
  public static final TalonFXConfiguration TOP_RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(50))
          .withSlot0(new Slot0Configs().withKP(0.5));
  public static final TalonFXConfiguration BOTTOM_LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(50));
  public static final TalonFXConfiguration BOTTOM_RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(50));

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreRPM",
          Map.entry(4.92, 1900.0),
          Map.entry(3.46, 1610.0),
          Map.entry(2.79, 1560.0),
          Map.entry(1.42, 1350.0));

  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringRegression", DISTANCE_TO_SCORE_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedingRPM",
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0),
          Map.entry(0.0, 0.0));

  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingRegression", DISTANCE_TO_FEEDING_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreToF",
          Map.entry(1.36, 1.017),
          Map.entry(2.42, 1.233),
          Map.entry(3.54, 1.148),
          Map.entry(5.5, 1.348));
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

  public static final double MAX_SAFE_RPM = 3000.0;
  public static final double IDLE_RPM = 40.0;
  public static double RPM_TOLERANCE = 60.0;
}
