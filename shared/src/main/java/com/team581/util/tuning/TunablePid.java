package com.team581.util.tuning;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.GlobalConfig;
import dev.doglog.DogLog;

public class TunablePid {
  /** Registers gains for slot 1 without changing another slot. */
  public static void register(String key, TalonFX motor, Slot1Configs config) {
    if (!GlobalConfig.IS_DEVELOPMENT) return;
    DogLog.tunable(
        key + "/kP", config.kP, value -> motor.getConfigurator().apply(config.withKP(value)));
    DogLog.tunable(
        key + "/kI", config.kI, value -> motor.getConfigurator().apply(config.withKI(value)));
    DogLog.tunable(
        key + "/kD", config.kD, value -> motor.getConfigurator().apply(config.withKD(value)));
    DogLog.tunable(
        key + "/kS", config.kS, value -> motor.getConfigurator().apply(config.withKS(value)));
    DogLog.tunable(
        key + "/kV", config.kV, value -> motor.getConfigurator().apply(config.withKV(value)));
    DogLog.tunable(
        key + "/kA", config.kA, value -> motor.getConfigurator().apply(config.withKA(value)));
    DogLog.tunable(
        key + "/kG", config.kG, value -> motor.getConfigurator().apply(config.withKG(value)));
  }

  /** Registers gains for slot 2 without changing another slot. */
  public static void register(String key, TalonFX motor, Slot2Configs config) {
    if (!GlobalConfig.IS_DEVELOPMENT) return;
    DogLog.tunable(
        key + "/kP", config.kP, value -> motor.getConfigurator().apply(config.withKP(value)));
    DogLog.tunable(
        key + "/kI", config.kI, value -> motor.getConfigurator().apply(config.withKI(value)));
    DogLog.tunable(
        key + "/kD", config.kD, value -> motor.getConfigurator().apply(config.withKD(value)));
    DogLog.tunable(
        key + "/kS", config.kS, value -> motor.getConfigurator().apply(config.withKS(value)));
    DogLog.tunable(
        key + "/kV", config.kV, value -> motor.getConfigurator().apply(config.withKV(value)));
    DogLog.tunable(
        key + "/kA", config.kA, value -> motor.getConfigurator().apply(config.withKA(value)));
    DogLog.tunable(
        key + "/kG", config.kG, value -> motor.getConfigurator().apply(config.withKG(value)));
  }

  public static void register(String key, TalonFX motor, TalonFXConfiguration defaultConfig) {
    if (!GlobalConfig.IS_DEVELOPMENT) {
      return;
    }

    DogLog.tunable(
        key + "/kP",
        defaultConfig.Slot0.kP,
        newP -> motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newP)));
    DogLog.tunable(
        key + "/kI",
        defaultConfig.Slot0.kI,
        newI -> motor.getConfigurator().apply(defaultConfig.Slot0.withKI(newI)));
    DogLog.tunable(
        key + "/kD",
        defaultConfig.Slot0.kD,
        newD -> motor.getConfigurator().apply(defaultConfig.Slot0.withKD(newD)));
    DogLog.tunable(
        key + "/kS",
        defaultConfig.Slot0.kS,
        newS -> motor.getConfigurator().apply(defaultConfig.Slot0.withKS(newS)));
    DogLog.tunable(
        key + "/kV",
        defaultConfig.Slot0.kV,
        newV -> motor.getConfigurator().apply(defaultConfig.Slot0.withKV(newV)));
    DogLog.tunable(
        key + "/kA",
        defaultConfig.Slot0.kA,
        newA -> motor.getConfigurator().apply(defaultConfig.Slot0.withKA(newA)));
    DogLog.tunable(
        key + "/kG",
        defaultConfig.Slot0.kG,
        newG -> motor.getConfigurator().apply(defaultConfig.Slot0.withKG(newG)));

    DogLog.tunable(
        key + "/MotionMagic/MaxVel",
        defaultConfig.MotionMagic.MotionMagicCruiseVelocity,
        newMaxVelocity ->
            motor
                .getConfigurator()
                .apply(defaultConfig.MotionMagic.withMotionMagicCruiseVelocity(newMaxVelocity)));
    DogLog.tunable(
        key + "/MotionMagic/MaxAccel",
        defaultConfig.MotionMagic.MotionMagicAcceleration,
        newMaxAccel ->
            motor
                .getConfigurator()
                .apply(defaultConfig.MotionMagic.withMotionMagicAcceleration(newMaxAccel)));
  }

  private TunablePid() {}
}
