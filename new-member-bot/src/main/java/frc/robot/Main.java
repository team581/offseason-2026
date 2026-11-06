package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.team581.GlobalConfig;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  public static void main(String... args) {
    RobotBase.startRobot(() -> {
      // Configure logging after HAL starts, before hardware and subsystems initialize.
      SignalLogger.enableAutoLogging(false);
      SignalLogger.stop();
      DogLog.setOptions(
          new DogLogOptions()
              .withCaptureDs(true)
              .withNtPublish(GlobalConfig.IS_DEVELOPMENT)
              .withNtTunables(GlobalConfig.IS_DEVELOPMENT)
              .withLogExtras(true)
              .withUseLogThread(false));
      return new Robot();
    });
  }

  private Main() {}
}
