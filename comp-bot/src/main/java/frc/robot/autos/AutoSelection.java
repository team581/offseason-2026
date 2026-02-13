package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import com.team581.trailblazer.Trailblazer;
import frc.robot.autos.auto_state_machines.DoNothingAuto;
import frc.robot.autos.auto_state_machines.IntegrationTest;
// import frc.robot.autos.auto_state_machines.TrenchRFeed2Auto;
import frc.robot.autos.auto_state_machines.RightStraightShootClimbAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING(DoNothingAuto::new),
 // TRENCH_FEED_2(TrenchRFeed2Auto::new),
  TRENCH_SHOOT_CLIMB(RightStraightShootClimbAuto::new),
 // TRENCH_SWOOP_INTAKE_SHOOT_CLIMB(),
  INTEGRATION_TEST(IntegrationTest::new);

  public final BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseImperativeAuto<?>> auto) {
    this.auto = auto;
  }
}
