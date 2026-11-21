package frc.robot.autos;

import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.robot_manager.RobotManager;
import java.util.concurrent.Executor;

/** Dependencies provided to every Trailblazer 3 auto routine. */
public record AutoContext(
    RobotManager robot, Trailblazer trailblazer, Alliance alliance, Executor pathExecutor) {}
