package frc.robot;

import java.lang.reflect.Field;

/** Benchmark-only helpers kept out of the production robot API. */
final class RobotBenchmarkUtil {
  private static Class<?> boxed(Class<?> type) {
    if (!type.isPrimitive()) {
      return type;
    }
    return switch (type.getName()) {
      case "boolean" -> Boolean.class;
      case "byte" -> Byte.class;
      case "char" -> Character.class;
      case "double" -> Double.class;
      case "float" -> Float.class;
      case "int" -> Integer.class;
      case "long" -> Long.class;
      case "short" -> Short.class;
      default -> type;
    };
  }

  private static Object field(Object owner, String name) {
    try {
      Field field = owner.getClass().getDeclaredField(name);
      field.setAccessible(true);
      return field.get(owner);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("Benchmark reflection failed for " + name, e);
    }
  }

  private static Object invoke(Object owner, String method, Object... args) {
    try {
      for (var candidate : owner.getClass().getMethods()) {
        if (candidate.getName().equals(method)
            && candidate.getParameterCount() == args.length
            && parametersMatch(candidate.getParameterTypes(), args)) {
          return candidate.invoke(owner, args);
        }
      }
      throw new NoSuchMethodException(method);
    } catch (ReflectiveOperationException | IllegalArgumentException e) {
      throw new IllegalStateException("Benchmark invocation failed for " + method, e);
    }
  }

  private static boolean parametersMatch(Class<?>[] parameterTypes, Object[] args) {
    for (int index = 0; index < parameterTypes.length; index++) {
      if (args[index] == null) {
        if (parameterTypes[index].isPrimitive()) {
          return false;
        }
      } else if (!boxed(parameterTypes[index]).isInstance(args[index])) {
        return false;
      }
    }
    return true;
  }

  private static void seedMotor(Object motor, double rotorVelocity) {
    Object state = invoke(motor, "getSimState");
    invoke(state, "setSupplyVoltage", 12.0);
    invoke(state, "setRotorVelocity", rotorVelocity);
  }

  private static Object value(Object owner, String fieldName, String method) {
    return invoke(field(owner, fieldName), method);
  }

  static String behaviorSnapshot(Robot robot) {
    return value(robot, "robotManager", "getState")
        + "|"
        + value(robot, "hopperManager", "getState")
        + "|"
        + value(robot, "swerve", "getState")
        + "|"
        + value(robot, "shooter", "getState")
        + "|"
        + value(robot, "shooterHood", "getState")
        + "|"
        + value(robot, "intake", "getState")
        + "|"
        + value(robot, "conveyor", "getState")
        + "|"
        + value(robot, "feeder", "getState")
        + "|"
        + value(robot, "deploy", "getState")
        + "|"
        + value(robot, "swerve", "getRequestedSpeeds")
        + "|"
        + value(robot, "shooterHood", "getAngle")
        + "|"
        + value(robot, "deploy", "getPosition")
        + "|"
        + value(robot, "localization", "getPose")
        + "|"
        + value(robot, "clusterMap", "getBestClusterLane")
        + "|"
        + value(robot, "clusterMap", "getBestClusterPose");
  }

  static void initializeSimulation(Robot robot) {
    Object hardware = field(robot, "hardware");
    Object drivetrain = field(hardware, "drivetrain");
    invoke(invoke(drivetrain, "getOdometryThread"), "stop");
  }

  static void simulationStep(Robot robot, double elapsedSeconds) {
    Object hardware = field(robot, "hardware");
    invoke(field(hardware, "drivetrain"), "updateSimState", 0.020, 12.0);
    double rotorVelocity = 5.0 * Math.sin(elapsedSeconds * 0.5);
    String[] motorNames = {
      "deployDifferentialMechanism",
      "intakeLeftMotor",
      "intakeRightMotor",
      "conveyorTopMotor",
      "conveyorBottomMotor",
      "feederTopMotor",
      "feederBottomMotor",
      "shooterHoodMotor",
      "shooterBottomLeftMotor",
      "shooterBottomRightMotor",
      "shooterTopLeftMotor",
      "shooterTopRightMotor"
    };
    for (String name : motorNames) {
      Object device = field(hardware, name);
      Object target =
          name.equals("deployDifferentialMechanism") ? invoke(device, "getLeader") : device;
      seedMotor(target, rotorVelocity);
      if (name.equals("deployDifferentialMechanism")) {
        seedMotor(invoke(device, "getFollower"), rotorVelocity);
      }
    }
    Object range = field(hardware, "hopperCANRange");
    invoke(invoke(range, "getSimState"), "setSupplyVoltage", 12.0);
    invoke(invoke(range, "getSimState"), "setDistance", 0.20 + 0.04 * Math.sin(elapsedSeconds));
  }

  private RobotBenchmarkUtil() {}
}
