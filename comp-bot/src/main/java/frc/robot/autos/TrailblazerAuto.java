package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.state_machines.StateMachine;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;

abstract class TrailblazerAuto<S extends Enum<S>> extends StateMachine<S> {
  protected final Trailblazer trailblazer;
  protected final Alliance alliance;

  protected TrailblazerAuto(S initialState, Trailblazer trailblazer, Alliance alliance) {
    super(initialState);
    this.trailblazer = trailblazer;
    this.alliance = alliance;
  }

  protected final void activate(AutoSegment segment) {
    trailblazer.activate(
        CompAutoField.segment(segment, alliance), CompAutoField.clearance(alliance));
  }
}
