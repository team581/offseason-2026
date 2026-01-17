package frc.robot.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class TurretCalculator {
    private static final Transform2d TURRET_TO_ROBOT = new Transform2d(Units.inchesToMeters(-0.5),0.0, Rotation2d.kZero);
        private static final double MIN_TURRET_ANGLE = -149.105;
    private static final double MAX_TURRET_ANGLE = 149.105;


    public static double calculateTurretAimingAngle(Pose2d robot, Pose2d target) {

         robot = robot.plus(TURRET_TO_ROBOT);
    var targetAngle =
        Units.radiansToDegrees(
            Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX()));
    var robotHeading = robot.getRotation().getDegrees();

    targetAngle = MathUtil.inputModulus(targetAngle, -180, 180);
    robotHeading = MathUtil.inputModulus(robotHeading, -180, 180);

    return MathUtil.inputModulus(targetAngle - robotHeading, -180, 180);
    }

     public static double calculateSwerveTurretCompensationAngle(double wantedTurretAngle, Pose2d robot) {
    var robotHeading = robot.getRotation().getDegrees();
    robotHeading = MathUtil.inputModulus(robotHeading, -180, 180);
wantedTurretAngle = MathUtil.inputModulus(wantedTurretAngle, -180, 180);
       var actualTargetRotation = MathUtil.inputModulus(wantedTurretAngle + robotHeading,-180,180);
       if (wantedTurretAngle> 0.0) {

           return MathUtil.inputModulus(actualTargetRotation - 30.0,-180,180) - MAX_TURRET_ANGLE;
       } else {
        return MathUtil.inputModulus(actualTargetRotation + 30.0,-180,180) - MIN_TURRET_ANGLE;
       }
    }
}
