package frc.robot.testing;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

/** Closed-loop field-frame control used only by test-mode routines and their desktop fixture. */
public final class TestDriveRequest extends SwerveRequest.FieldCentric {
  public TestDriveRequest() {
    withDeadband(0.07);
    withRotationalDeadband(0.05);
    withDriveRequestType(DriveRequestType.Velocity);
    withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);
  }
}
