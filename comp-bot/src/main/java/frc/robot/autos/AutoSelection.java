package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.LeftCircleAlternateAuto;
import frc.robot.autos.auto_state_machines.LeftCircleAuto;
import frc.robot.autos.auto_state_machines.LeftIntegratedAuto;
import frc.robot.autos.auto_state_machines.RightCircleAlternateAuto;
import frc.robot.autos.auto_state_machines.RightCircleAuto;
import frc.robot.autos.auto_state_machines.RightIntegratedAuto;
// import frc.robot.autos.auto_state_machines.RightPullSwoopShootAuto;
import frc.robot.autos.auto_state_machines.RightStraightShootClimbAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  RIGHT_INTEGRATED_AUTO(RightIntegratedAuto::new),
  LEFT_INTEGRATED_AUTO(LeftIntegratedAuto::new),
  RIGHT_STRAIGHT_SHOOT(RightStraightShootClimbAuto::new),
  //  RIGHT_PULL_SWOOP_SHOOT(RightPullSwoopShootAuto::new),
  RIGHT_CIRCLE_SOM(RightCircleAuto::new),
  RIGHT_ALTERNATE_CIRCLE_SOM(RightCircleAlternateAuto::new),
  LEFT_CIRCLE_SOM(LeftCircleAuto::new),
  LEFT_ALTERNATE_CIRCLE_SOM(LeftCircleAlternateAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
