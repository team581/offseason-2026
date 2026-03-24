package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.ClusterLaneDemo;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.LeftCircleAlternateAuto;
import frc.robot.autos.auto_state_machines.LeftCircleSoMAuto;
import frc.robot.autos.auto_state_machines.RightCircleAlternateAuto;
import frc.robot.autos.auto_state_machines.RightCircleSoMAuto;
import frc.robot.autos.auto_state_machines.RightIntegratedAuto;
// import frc.robot.autos.auto_state_machines.RightPullSwoopShootAuto;
import frc.robot.autos.auto_state_machines.RightStraightShootClimbAuto;
import frc.robot.autos.auto_state_machines.RightTestPathAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
  CLUSTER_LANE_DEMO(ClusterLaneDemo::new),
  RIGHT_STRAIGHT_SHOOT(RightStraightShootClimbAuto::new),
  //  RIGHT_PULL_SWOOP_SHOOT(RightPullSwoopShootAuto::new),
  TEST_AUTO_PATH(RightTestPathAuto::new),
  RIGHT_INTEGRATED_AUTO(RightIntegratedAuto::new),
  RIGHT_CIRCLE_SOM(RightCircleSoMAuto::new),
  RIGHT_ALTERNATE_CIRCLE_SOM(RightCircleAlternateAuto::new),
  LEFT_CIRCLE_SOM(LeftCircleSoMAuto::new),
  LEFT_ALTERNATE_CIRCLE_SOM(LeftCircleAlternateAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
