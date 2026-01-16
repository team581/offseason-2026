package com.team581.controller;

import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;

public class RumbleController extends StateMachineSubsystem<RumbleControllerState> {
  public static final double MATCH_DURATION_TELEOP = 135;

  private final Timer matchTimer = new Timer();
  private final GenericHID controller;
  private final boolean matchTimeRumble;

  private boolean didMatchTimeRumble90 = false;
  private boolean didMatchTimeRumble60 = false;
  private boolean didMatchTimeRumble30 = false;

  @Override
  public void teleopInit() {
    matchTimer.reset();
    matchTimer.start();
    didMatchTimeRumble90 = false;
    didMatchTimeRumble60 = false;
    didMatchTimeRumble30 = false;
  }

  @Override
  public void disabledInit() {
    matchTimer.stop();
  }

  public RumbleController(
      GenericHID controller,
      boolean matchTimeRumble,
      SubsystemPriorityBase rumbleControllerPriority) {
    super(rumbleControllerPriority, RumbleControllerState.OFF);
    this.controller = controller;
    this.matchTimeRumble = matchTimeRumble;
  }

  public void rumbleRequest() {
    if (!DriverStation.isAutonomous()) {
      setStateFromRequest(RumbleControllerState.ON);
      resetTimeout();
    }
  }

  @Override
  protected void whileInState(RumbleControllerState state) {
    if (!matchTimeRumble) {
      return;
    }

    if (!didMatchTimeRumble90) {
      rumbleRequest();
      didMatchTimeRumble90 = true;
    }
    if (!didMatchTimeRumble60) {
      rumbleRequest();
      didMatchTimeRumble60 = true;
    }
    if (!didMatchTimeRumble30) {
      rumbleRequest();
      didMatchTimeRumble30 = true;
    }
  }

  @Override
  protected RumbleControllerState getNextState(RumbleControllerState currentState) {
    return switch (currentState) {
      case ON -> timeout(0.5) ? RumbleControllerState.OFF : currentState;
      case OFF -> currentState;
    };
  }

  @Override
  protected void afterTransition(RumbleControllerState newState) {
    switch (newState) {
      case ON -> controller.setRumble(RumbleType.kBothRumble, 1);
      case OFF -> controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
