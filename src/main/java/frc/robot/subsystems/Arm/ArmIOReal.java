// Copyright 2025-2026 FRC 6985
// https://www.enkatech6985.com/
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Arm.MirrorType;
import frc.robot.Constants.Arm.PivotState;
import frc.robot.Constants.Arm.RollerState;
import frc.robot.Constants.Arm.Side;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Utils;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Robot Arm Subsystem. The Kotlin 'object Arm' structure is implemented as a Singleton pattern in
 * Java.
 */
public class ArmIOReal implements ArmIO {

  // --- HARDWARE AND CONSTANT PROPERTIES ---
  private final CANcoder absoluteEncoder = new CANcoder(Constants.CanIds.armEncoder);
  private final TalonFX armPivotMotor = new TalonFX(Constants.CanIds.ArmPivotMotor);
  private final TalonFX rollerMotor = new TalonFX(Constants.CanIds.ArmRollerMotor);
  private final StatusSignal<Current> statorCurrentSignal = rollerMotor.getStatorCurrent();

  private final InterpolatingDoubleTreeMap elevatorToArm = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap elevatorToArmWhenIntakeDown =
      new InterpolatingDoubleTreeMap();

  private final double deadzoneAngle = Math.toRadians(20.0);

  private final Debouncer coralCurrentDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kBoth);
  private final Debouncer algaeCurrentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  private final Timer autoTimer = new Timer();

  public final double armOffsetIncrementRadians = Math.toRadians(0.1);

  // --- MUTABLE STATES ---
  private PivotState pivotState = PivotState.Up;
  private RollerState rollerState = RollerState.Off;
  private boolean zeroed = false;
  private boolean hasObject = false;
  public boolean isArmStuck = false;
  private double armOffsetRadians = 0.0;
  private long lastUpdatedTick = -1;
  private double lastCachedValue = 0.0;
  private final RobotContainer rC;
  private final Drive drive;

  public ArmIOReal(RobotContainer rc) {
    this.rC = rc;
    this.drive = rc.subsystems.drive;

    armPivotMotor.getConfigurator().apply(Constants.Arm.pivotConfig);
    rollerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(80.0));
    statorCurrentSignal.setUpdateFrequency(100.0);

    // Populate interpolating maps
    for (double[] pair : Constants.ARM_ELEVATOR_PAIRS) {
      elevatorToArm.put(pair[1], pair[0]);
    }
    for (double[] pair : Constants.ARM_INTERPOLATION_INTAKE_DOWN) {
      elevatorToArmWhenIntakeDown.put(pair[1], pair[0]);
    }
  }

  // --- GETTER METHODS (Kotlin properties) ---

  public double getPosition() {
    // armPivotMotor.position.valueAsDouble * 2*Math.PI + armOffsetRadians
    return armPivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI + armOffsetRadians;
  }

  public boolean atSetpoint() {
    return Math.abs(getDesiredPosition() - getPosition()) < Constants.Arm.SETPOINT_THRESHOLD;
  }

  public boolean getInsideFrame() {
    double positionMod = MathUtil.angleModulus(getPosition());
    return Math.abs(positionMod) < Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE
        || Math.abs(positionMod) > (Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
  }

  public boolean getUndebouncedHasObject() {
    double current = statorCurrentSignal.getValueAsDouble();
    if (rollerState == RollerState.Idle || rollerState == RollerState.SlowIdle) {
      return current > Constants.Arm.IDLE_CURRENT_DRAW;
    } else {
      return current > Constants.Arm.CURRENT_DRAW;
    }
  }

  // --- POSITION AND SAFETY METHODS ---

  public Side getSideCloserToReef() {
    Rotation2d directionTowardReefCenter =
        (Utils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER, rC)
                .minus(drive.poseEstimator.getEstimatedPosition().getTranslation()))
            .getAngle();
    Rotation2d directionTowardRight =
        drive.poseEstimator.getEstimatedPosition().getRotation().rotateBy(Rotation2d.kCW_90deg);

    // Uses dot product formula (for unit vectors). This is always between 0 and 180
    // degrees (0 and pi radians).
    double ang =
        Math.acos(
            directionTowardReefCenter.getCos() * directionTowardRight.getCos()
                + directionTowardReefCenter.getSin() * directionTowardRight.getSin());

    if (Math.PI / 2 - deadzoneAngle < ang && ang < Math.PI / 2 + deadzoneAngle) {
      return Side.Neither;
    } else if (ang < Math.PI / 2) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public Side getSideCloserToBarge() {
    boolean isOnBlue = !rC.isOnRedSide();
    double rotation = drive.poseEstimator.getEstimatedPosition().getRotation().getRadians();

    if ((rotation < Math.PI && rotation > Math.PI - deadzoneAngle)
        || (rotation > -Math.PI && rotation < -Math.PI + deadzoneAngle)
        || (rotation > -deadzoneAngle && rotation < deadzoneAngle)) {
      return Side.Neither;
    } else if ((isOnBlue && rotation > 0.0) || (!isOnBlue && rotation < 0.0)) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public Side getSideCloserToProcessor() {
    boolean isOnBlue = !rC.isOnRedSide();
    double rotation = drive.poseEstimator.getEstimatedPosition().getRotation().getRadians();

    if ((rotation < (Math.PI / 2 + deadzoneAngle) && rotation > (Math.PI / 2 - deadzoneAngle))
        || (rotation > (-Math.PI / 2 - deadzoneAngle)
            && rotation < (-Math.PI / 2 + deadzoneAngle))) {
      return Side.Neither;
    } else if ((isOnBlue && rotation < Math.PI / 2 && rotation > -Math.PI / 2)
        || (!isOnBlue && (rotation > Math.PI / 2 || rotation < -Math.PI / 2))) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public boolean atSafeReefDistance() {
    return drive
            .poseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .getDistance(Utils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER, rC))
        > Constants.Arm.SAFE_DISTANCE_FROM_REEF_CENTER;
  }

  public double getDesiredPosition() {
    // Translation of the 'desiredPosition' Kotlin property with memoization
    // (caching)
    if (lastUpdatedTick == rC.tickNumber) {
      return lastCachedValue;
    }
    double answer;
    // long startTime = System.currentTimeMillis();

    answer =
        positionFromAngle(
            pivotState.getDesiredAngle(this),
            pivotState.mirrorType != MirrorType.ActuallyFixedAngle);

    // long milliseconds = System.currentTimeMillis() - startTime;
    // lastUpdatedTick = rC.tickNumber;
    // lastCachedValue = answer;
    // if (milliseconds > 5)
    // System.out.println("desired position took " + milliseconds + " ms"); //
    // Orijinal println'ı koru
    return answer;
  }

  public double positionFromAngle(double angle, boolean respectReef) {
    // Potential rotation positions (one in 0 to 2*PI, one in -2*PI to 0)
    List<Double> positions =
        Stream.of(Utils.wrapTo0_2PI(angle), Utils.wrapTo0_2PI(angle) - 2 * Math.PI)
            .filter(
                p ->
                    (p >= Constants.Arm.ALLOWED_OPERATING_RANGE_MIN
                        && p <= Constants.Arm.ALLOWED_OPERATING_RANGE_MAX)) // Check
            // allowed
            // range
            .collect(Collectors.toList());

    final double actualArmPosition = getPosition();
    final Side closeSide = getSideCloserToReef();

    double p;
    if (positions.size() == 1) {
      p = positions.get(0);
    } else if (respectReef) {
      switch (closeSide) {
        case Neither:
          // Choose whichever position is closest to current angle
          p =
              positions.stream()
                  .min(
                      (p1, p2) ->
                          Double.compare(
                              Math.abs(p1 - actualArmPosition), Math.abs(p2 - actualArmPosition)))
                  .orElse(0.0);
          break;
        case Right:
          // Position selection based on direction of rotation near the reef (simplified)
          p = (actualArmPosition < Math.PI / 2) ? positions.get(1) : positions.get(0);
          break;
        case Left:
          // Position selection based on direction of rotation near the reef (simplified)
          p = (actualArmPosition > -Math.PI / 2) ? positions.get(0) : positions.get(1);
          break;
        default:
          p = 0.0;
      }
    } else {
      // Choose the closest position (ignoring reef for now)
      p =
          positions.stream()
              .min(
                  (p1, p2) ->
                      Double.compare(
                          Math.abs(p1 - actualArmPosition), Math.abs(p2 - actualArmPosition)))
              .orElse(0.0);
    }

    // Reef Safety Constraint (Prevent hitting the outer frame)
    // This works because the only time we should be pivoting to a position below 90
    // degrees is when going to rest state.
    final boolean notAtSafeReefDistance = !atSafeReefDistance();
    if (actualArmPosition > Math.PI / 2
        && p < Math.PI / 2
        && closeSide == Side.Right
        && notAtSafeReefDistance) {
      isArmStuck = true;
      p = Math.max(p, Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
    } else if (actualArmPosition < -Math.PI / 2
        && p > -Math.PI / 2
        && closeSide == Side.Left
        && notAtSafeReefDistance) {
      isArmStuck = true;
      p = Math.min(p, -Math.PI + Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
    } else {
      isArmStuck = false;
    }

    // Elevator Clamping (Prevent arm hitting the elevator)
    // CLAMPING, DO NOT MOVE OR REMOVE!
    double actualElevatorHeight =
        rC.subsystems.elevator.getHeight(); // Assume Elevator is a Singleton

    InterpolatingDoubleTreeMap interpolationMap =
        (rC.subsystems.intake.getPivotState() == Constants.Intake.PivotState.Down
                && rC.subsystems.intake.atSetpoint())
            ? elevatorToArmWhenIntakeDown
            : elevatorToArm;

    // This value is actually measured in radians DOWN from straight-up (Math.PI)
    double limit = interpolationMap.get(actualElevatorHeight);

    if (MathUtil.isNear(Math.PI, limit, 0.0001)) {
      return p; // mitigates IEEE754 issues
    } else if (actualArmPosition < 0.0) {
      // Negative arm angles
      return MathUtil.clamp(p, -Math.PI - limit, -Math.PI + limit);
    } else {
      // Positive arm angles
      return MathUtil.clamp(p, Math.PI - limit, Math.PI + limit);
    }
  }

  // --- SYSTEM COMMAND METHODS ---

  public void setState(PivotState pivot, RollerState rollers) {
    this.pivotState = pivot;
    this.rollerState = rollers;
  }

  public void setCoastEnabled(boolean coast) {
    if (coast) armPivotMotor.setNeutralMode(NeutralModeValue.Coast);
    else armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  // For when the arm skips.
  public void offsetArm(double r) {
    armOffsetRadians += r;
  }

  public double closeClampedPosition() {
    double x =
        absoluteEncoder.getPosition().getValueAsDouble()
            - Constants.Arm.PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS;
    // Wrap encoder reading to be between -0.5 and 0.5 rotations
    while (x < -0.5) x += 1.0;
    while (x > 0.5) x -= 1.0;
    double rawReadingArmRotations = x * Constants.Arm.PIVOT_ENCODER_RATIO;

    final double allowedOffsetArmRotations = 12.0 / 360.0;

    // Hack for clamping to a known limit
    if (Math.abs(rawReadingArmRotations - 0.5) < allowedOffsetArmRotations) {
      return 0.5;
    } else if (Math.abs(rawReadingArmRotations - (-0.5)) < allowedOffsetArmRotations) {
      return -0.5;
    } else {
      return rawReadingArmRotations;
    }
  }

  public void resetRelativeFromAbsolute() {
    armPivotMotor.setPosition(closeClampedPosition());
    zeroed = true;
  }

  // --- PERIODIC METHOD ---

  @Override
  public void periodic() {
    // Arm Offset Control
    // printlns commented out as in original Kotlin code
    // // println("negative ${Controls.wantOffsetArmNegative} positive
    // ${Controls.wantOffsetArmPositive}")

    // ! FIXME: Controls class not defined
    // if (Controls.wantOffsetArmPositive)
    // offsetArm(armOffsetIncrementRadians);
    // if (Controls.wantOffsetArmNegative)
    // offsetArm(-armOffsetIncrementRadians);

    // Object Detection
    final boolean atStartOfAuto = (rC.robot.isAutonomous() && autoTimer.get() < 0.75);
    statorCurrentSignal.refresh();

    boolean undebounced = getUndebouncedHasObject();
    boolean debouncedHasCoral = coralCurrentDebouncer.calculate(undebounced);
    boolean debouncedHasAlgae = algaeCurrentDebouncer.calculate(undebounced);

    hasObject =
        atStartOfAuto
            || (rollerState == RollerState.AlgaeIdle ? debouncedHasAlgae : debouncedHasCoral);

    if (!isZeroed() || !rC.subsystems.elevator.isZeroed()) // Assume Elevator is a Singleton
    return;

    // Roller Command
    rollerMotor.set((atStartOfAuto ? RollerState.FastIdle.dutyCycle : rollerState.dutyCycle));

    // Pivot Control (Motion Magic and Gravity Feedforward)
    double positionSetpoint = getDesiredPosition();
    double gravityFeedforward = Constants.Arm.POSITION_DEPENDENT_KG * Math.sin(getPosition());

    // TalonFX MotionMagicVoltage command, converting angle (radians) to rotation:
    // (setpoint - offset) / (2*PI)
    armPivotMotor.setControl(
        new MotionMagicVoltage((positionSetpoint - armOffsetRadians) / (2 * Math.PI))
            .withFeedForward(gravityFeedforward));
  }

  @Override
  public boolean isZeroed() {
    return zeroed;
  }

  @Override
  public void setZeroed(boolean z) {
    this.zeroed = z;
  }
  // --- SENDABLE LOGGING ---

  public void initSendable(SendableBuilder builder) {
    // Kotlin lambdas are translated to Java lambda expressions: () -> expression
    builder.addDoubleProperty("Arm offset (deg)", () -> Math.toDegrees(armOffsetRadians), null);
    builder.addDoubleProperty("Arm position (deg)", () -> Math.toDegrees(getPosition()), null);
    builder.addDoubleProperty(
        "Raw Encoder", () -> absoluteEncoder.getPosition().getValueAsDouble(), null);
    // // builder.addDoubleProperty("Clamped position encoder (deg)", {
    // closeClampedPosition() * 360.0 }, {})
    builder.addBooleanProperty("Has object?", () -> hasObject, null);
    // // builder.addBooleanProperty("Undebounced has object?", {
    // undebouncedHasObject}, {})
    builder.addDoubleProperty(
        "Desired position (deg)", () -> Math.toDegrees(getDesiredPosition()), null);
    builder.addDoubleProperty(
        "Motion magic setpoint (deg)",
        () -> 360.0 * armPivotMotor.getClosedLoopReference().getValueAsDouble(),
        null);
    // // builder.addDoubleProperty("Before-safety desired angle (deg)",
    // {Math.toDegrees(pivotState.desiredAngle)}, {})
    // // builder.addStringProperty("Arm pivot state", { pivotState.toString() },
    // {})
    builder.addBooleanProperty("At setpoint?", () -> atSetpoint(), null);
    // // builder.addDoubleProperty("Zeroing Position", { closeClampedPosition() *
    // 360.0 }, {})
    // // builder.addDoubleProperty("angle limit", {
    // elevatorToArm.get(Elevator.height)}, {})
    builder.addDoubleProperty("roller current", () -> statorCurrentSignal.getValueAsDouble(), null);
    // Assume Utils.addClosedLoopProperties is available in Java
    // Utils.addClosedLoopProperties("Arm pivot", armPivotMotor, builder);
    // // builder.addBooleanProperty("Coast enabled?", { Robot.wasCoastModeEnabled
    // }, {})
  }
}
