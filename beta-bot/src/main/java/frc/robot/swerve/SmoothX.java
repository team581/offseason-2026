package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class SmoothX implements SwerveRequest {
  private double VelocityLimit = 0.75;
  private double lastAppliedLimit = -1.0; // Track the current hardware state

  private final MotionMagicConfigs m_configs = new MotionMagicConfigs();

  @Override
  public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modules) {
    double[] targets = {0.125, -0.125, -0.125, 0.125};

    // Only talk to the Configurator if the limit has changed
    boolean needsUpdate = (VelocityLimit != lastAppliedLimit);

    for (int i = 0; i < modules.length; ++i) {
      TalonFX steerMotor = (TalonFX) modules[i].getSteerMotor();

      if (needsUpdate) {
        steerMotor.getConfigurator().refresh(m_configs);
        m_configs.MotionMagicCruiseVelocity = VelocityLimit;
        steerMotor.getConfigurator().apply(m_configs);
      }

      // Standard control calls are NOT blocking, so these are fine to run every loop
      steerMotor.setControl(new MotionMagicExpoVoltage(targets[i]));
      modules[i].getDriveMotor().setControl(new NeutralOut());
    }

    // Update our tracker so we don't run the config logic next frame
    if (needsUpdate) {
      lastAppliedLimit = VelocityLimit;
    }

    return StatusCode.OK;
  }
}
