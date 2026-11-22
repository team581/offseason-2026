package com.team581.util.tuning;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.GlobalConfig;
import dev.doglog.DogLog;
import edu.wpi.first.units.Unit;
import java.util.List;
import java.util.function.Consumer;

public class TunablePid {
  /** Registers one set of slot 1 tunables for motors that share the same slot 1 config. */
  public static void register(String key, Slot1Configs config, TalonFX... motors) {
    if (!GlobalConfig.IS_DEVELOPMENT) return;
    registerGains(key, SlotConfigs.from(config), List.of(motors));
  }

  /** Registers one set of slot 2 tunables for motors that share the same slot 2 config. */
  public static void register(String key, Slot2Configs config, TalonFX... motors) {
    if (!GlobalConfig.IS_DEVELOPMENT) return;
    registerGains(key, SlotConfigs.from(config), List.of(motors));
  }

  /** Registers gains for slot 1 without changing another slot. */
  public static void register(String key, TalonFX motor, Slot1Configs config) {
    register(key, config, motor);
  }

  /** Registers gains for slot 2 without changing another slot. */
  public static void register(String key, TalonFX motor, Slot2Configs config) {
    register(key, config, motor);
  }

  public static void register(String key, TalonFX motor, TalonFXConfiguration defaultConfig) {
    register(key, defaultConfig, motor);
  }

  /**
   * Registers one set of tunables for motors that share the same slot 0 and Motion Magic configs.
   * No other settings from {@code defaultConfig} are applied to the motors.
   */
  public static void register(String key, TalonFXConfiguration defaultConfig, TalonFX... motors) {
    register(key, defaultConfig, RotationsPerSecond, RotationsPerSecondPerSecond, motors);
  }

  /** Registers tunables with the units already used by the configured Motion Magic limits. */
  public static void register(
      String key,
      TalonFXConfiguration defaultConfig,
      Unit velocityUnit,
      Unit accelerationUnit,
      TalonFX... motors) {
    if (!GlobalConfig.IS_DEVELOPMENT) return;

    var targets = List.of(motors);
    registerGains(key, SlotConfigs.from(defaultConfig.Slot0), targets);

    var motionMagic = defaultConfig.MotionMagic.clone();
    Consumer<MotionMagicConfigs> apply =
        updated -> targets.forEach(motor -> motor.getConfigurator().apply(updated));

    DogLog.tunable(
        key + "/MotionMagic/MaxVel",
        motionMagic.MotionMagicCruiseVelocity,
        velocityUnit,
        value -> apply.accept(motionMagic.withMotionMagicCruiseVelocity(value)));
    DogLog.tunable(
        key + "/MotionMagic/MaxAccel",
        motionMagic.MotionMagicAcceleration,
        accelerationUnit,
        value -> apply.accept(motionMagic.withMotionMagicAcceleration(value)));
  }

  private static void registerGains(String key, SlotConfigs config, List<TalonFX> motors) {
    Consumer<SlotConfigs> apply =
        updated -> motors.forEach(motor -> motor.getConfigurator().apply(updated));

    DogLog.tunable(key + "/kP", config.kP, value -> apply.accept(config.withKP(value)));
    DogLog.tunable(key + "/kI", config.kI, value -> apply.accept(config.withKI(value)));
    DogLog.tunable(key + "/kD", config.kD, value -> apply.accept(config.withKD(value)));
    DogLog.tunable(key + "/kS", config.kS, value -> apply.accept(config.withKS(value)));
    DogLog.tunable(key + "/kV", config.kV, value -> apply.accept(config.withKV(value)));
    DogLog.tunable(key + "/kA", config.kA, value -> apply.accept(config.withKA(value)));
    DogLog.tunable(key + "/kG", config.kG, value -> apply.accept(config.withKG(value)));
  }

  private TunablePid() {}
}
