package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.Field;
import frc.robot.RobotContainer;

public final class Utils {

  private Utils() {}

  // ----------------------------------------
  // Deadzone utilities
  // ----------------------------------------

  /** Fancy deadzone: scales input from 0.0 -> 1.0 */
  public static double deadZone(double input, double zone) {
    if (Math.abs(input) < zone) return 0.0;
    if (input > 1.0) return 1.0;
    if (input < -1.0) return -1.0;
    return (input - zone) / (1.0 - zone);
  }

  /** Returns input untouched if outside deadzone, 0 if inside */
  public static double unclampedDeadzone(double input, double zone) {
    return Math.abs(input) < zone ? 0.0 : input;
  }

  // ----------------------------------------
  // Angle wrapping
  // ----------------------------------------
  public static double wrapTo0_2PI(double a) {
    double mod = MathUtil.angleModulus(a); // wraps -PI to +PI
    return mod < 0 ? mod + 2 * Math.PI : mod;
  }

  // ----------------------------------------
  // Field-bound check
  // ----------------------------------------
  public static boolean isInsideField(Translation2d t) {
    return t.getX() > 0.0
        && t.getY() > 0.0
        && t.getX() < Constants.Field.FIELD_X_SIZE
        && t.getY() < Constants.Field.FIELD_Y_SIZE;
  }

  // ----------------------------------------
  // Mirror utilities
  // ----------------------------------------
  public static Translation2d mirror(Translation2d t) {
    return new Translation2d(Constants.Field.FIELD_X_SIZE - t.getX(), t.getY());
  }

  public static Rotation2d mirror(Rotation2d r) {
    return new Rotation2d(Math.PI).minus(r);
  }

  public static Pose2d mirrorPose(Pose2d p) {
    return new Pose2d(
        new Translation2d(Field.FIELD_X_SIZE - p.getX(), p.getY()),
        new Rotation2d(-p.getRotation().getRadians()));
  }

  public static Pose2d mirror(Pose2d p) {
    return new Pose2d(mirror(p.getTranslation()), mirror(p.getRotation()));
  }

  // Mirror if red alliance (pass RobotContainer or Robot instance)
  public static Translation2d mirrorIfRed(Translation2d t, RobotContainer rC) {
    return rC.isRedAlliance() ? mirror(t) : t;
  }

  public static Rotation2d mirrorIfRed(Rotation2d r, RobotContainer rC) {
    return rC.isRedAlliance() ? mirror(r) : r;
  }

  public static Pose2d mirrorIfRed(Pose2d p, RobotContainer rC) {
    return rC.isRedAlliance() ? mirror(p) : p;
  }

  // ----------------------------------------
  // SendableBuilder / TalonFX utilities
  // ----------------------------------------
  public static void addClosedLoopProperties(String name, TalonFX motor, SendableBuilder builder) {
    builder.addDoubleProperty(
        name + " voltage (V)", () -> motor.getMotorVoltage().getValueAsDouble(), (__) -> {});
    // builder.addDoubleProperty(name + " supply Current (A)", () ->
    // motor.getSupplyCurrent().getValueAsDouble(), () -> {});
    // builder.addDoubleProperty(name + " stator Current (A)", () ->
    // motor.getStatorCurrent().getValueAsDouble(), () -> {});
    builder.addDoubleProperty(
        name + " raw velocity", () -> motor.getVelocity().getValueAsDouble(), (__) -> {});
    builder.addDoubleProperty(
        name + " raw acceleration", () -> motor.getAcceleration().getValueAsDouble(), (__) -> {});
    // builder.addDoubleProperty(name + " position", () ->
    // motor.getPosition().getValueAsDouble(), () -> {});
    // builder.addDoubleProperty(name + " motion magic setpoint*360", () ->
    // motor.getClosedLoopReference().getValueAsDouble()*360.0, () -> {});
  }
}
