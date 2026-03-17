package frc.robot.climber;

public class StubClimber extends GenericClimber {
  @Override
  public boolean atGoal() {
    return true;
  }

  @Override
  public double getHeight() {
    return getState().height;
  }

  @Override
  public void l1HangingRequest() {
    setStateFromRequest(ClimberState.L1_HANG);
  }

  @Override
  public void l1LineupRequest() {
    setStateFromRequest(ClimberState.L1_LINEUP);
  }

  @Override
  public void l2HangingRequest() {
    setStateFromRequest(ClimberState.L2_HANG);
  }

  @Override
  public void l2LineupRequest() {
    setStateFromRequest(ClimberState.L2_LINEUP);
  }

  @Override
  public void l3HangingRequest() {
    setStateFromRequest(ClimberState.L3_HANG);
  }

  @Override
  public void l3LineupRequest() {
    setStateFromRequest(ClimberState.L3_LINEUP);
  }

  @Override
  public void stowRequest() {
    setStateFromRequest(ClimberState.STOWED);
  }
}
