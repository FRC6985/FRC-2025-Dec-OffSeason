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
import frc.robot.Constants.Field;
import frc.robot.Constants.Arm.MirrorType;
import frc.robot.Constants.Arm.PivotState;
import frc.robot.Constants.Arm.RollerState;
import frc.robot.Constants.Arm.Side;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Utils;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class ArmIOReal implements ArmIO {

  private final CANcoder absoluteEncoder = new CANcoder(Constants.CanIds.armEncoder);
  private final TalonFX armPivotMotor = new TalonFX(Constants.CanIds.ArmPivotMotor);
  private final TalonFX rollerMotor = new TalonFX(Constants.CanIds.ArmRollerMotor);
  private final StatusSignal<Current> statorCurrentSignal = rollerMotor.getStatorCurrent();;

  // Durum Değişkenleri
  // --------------------------------------------------------------------------------

  private PivotState pivotState = PivotState.Up;
  private RollerState rollerState = RollerState.Off;
  private boolean isZeroed = false;
  private final Debouncer coralCurrentDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kBoth);
  private final Debouncer algaeCurrentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  private boolean hasObject = false;
  private final Timer autoTimer = new Timer();
  private final double armOffsetIncrementRadians = Math.toRadians(0.1);
  private double armOffsetRadians = 0.0;
  private boolean isArmStuck = false;
  private RobotContainer rC;
  private final Drive drive;
  // Cache için
  private long lastUpdatedTick = -1;
  private double lastCachedValue = 0.0;

  // İnterpolasyon Haritaları
  // --------------------------------------------------------------------------------

  private final InterpolatingDoubleTreeMap elevatorToArm = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap elevatorToArmWhenIntakeDown = new InterpolatingDoubleTreeMap();

  public InterpolatingDoubleTreeMap getElevatorToArm() {
    return elevatorToArm;
  }

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

  // Deadzone
  private final double deadzoneAngle = Math.toRadians(20.0);

  // Getter Metotları
  // --------------------------------------------------------------------------------

  public PivotState getPivotState() {
    return pivotState;
  }

  public RollerState getRollerState() {
    return rollerState;
  }

  public boolean isZeroed() {
    return isZeroed;
  }

  public double getArmOffsetRadians() {
    return armOffsetRadians;
  }

  public boolean getHasObject() {
    return hasObject;
  }

  public boolean isArmStuck() {
    return isArmStuck;
  }

  public double getPosition() {
    return armPivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI + armOffsetRadians;
  }

  public boolean getAtSetpoint() {
    return Math.abs(getDesiredPosition() - getPosition()) < Constants.Arm.SETPOINT_THRESHOLD;
  }

  public boolean isInsideFrame() {
    return Math.abs(MathUtil.angleModulus(getPosition())) < Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE ||
        Math.abs(MathUtil.angleModulus(getPosition())) > (Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
  }

  // Konum Hesaplamaları
  // --------------------------------------------------------------------------------

  public Side getSideCloserToReef() {
    Rotation2d directionTowardReefCenter = (Utils.mirrorIfRed(Field.BLUE_REEF_CENTER, rC)
        .minus(drive.poseEstimator.getEstimatedPosition().getTranslation())).getAngle();
    Rotation2d directionTowardRight = drive.poseEstimator.getEstimatedPosition().getRotation()
        .rotateBy(Rotation2d.kCW_90deg);

    // Uses dot product formula (for unit vectors). This is always between 0 and 180
    // degrees (0 and pi radians).
    // (r1.cos*r2.cos + r1.sin*r2.sin)
    double ang = Math.acos(directionTowardReefCenter.getCos() * directionTowardRight.getCos() +
        directionTowardReefCenter.getSin() * directionTowardRight.getSin());
    assert ang >= 0.0; // the math works out this way

    if (Math.PI / 2 - deadzoneAngle < ang && ang < Math.PI / 2 + deadzoneAngle) {
      return Side.Neither;
    } else if (ang < Math.PI / 2) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public Side getSideCloserToBarge() {
    final boolean isOnBlue = !rC.isOnRedSide();
    final double rotation = drive.poseEstimator.getEstimatedPosition().getRotation().getRadians();

    if ((rotation < Math.PI && rotation > Math.PI - deadzoneAngle) ||
        (rotation > -Math.PI && rotation < -Math.PI + deadzoneAngle) ||
        (rotation > -deadzoneAngle && rotation < deadzoneAngle)) {
      return Side.Neither;
    } else if ((isOnBlue && rotation > 0.0) || (!isOnBlue && rotation < 0.0)) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public Side getSideCloserToProcessor() {
    final boolean isOnBlue = !rC.isOnRedSide();
    final double rotation = drive.poseEstimator.getEstimatedPosition().getRotation().getRadians();

    if ((rotation < (Math.PI / 2 + deadzoneAngle) && rotation > (Math.PI / 2 - deadzoneAngle)) ||
        (rotation > (-Math.PI / 2 - deadzoneAngle) && rotation < (-Math.PI / 2 + deadzoneAngle))) {
      return Side.Neither;
    } else if ((isOnBlue && rotation < Math.PI / 2 && rotation > -Math.PI / 2) ||
        (!isOnBlue && (rotation > Math.PI / 2 || rotation < -Math.PI / 2))) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public boolean atSafeReefDistance() {
    return drive.poseEstimator.getEstimatedPosition().getTranslation().getDistance(
        Utils.mirrorIfRed(Field.BLUE_REEF_CENTER, rC)) > Constants.Arm.SAFE_DISTANCE_FROM_REEF_CENTER;
  }

  public boolean atSafePlacementDistance() {
    return drive.poseEstimator.getEstimatedPosition().getTranslation().getDistance(
        Utils.mirrorIfRed(Field.BLUE_REEF_CENTER, rC)) > Constants.Arm.SAFE_PLACEMENT_DISTANCE;
  }

  public boolean atSafeBargeDistance() {
    return drive.poseEstimator.getEstimatedPosition().getX() < Constants.Field.FIELD_X_SIZE / 2
        - Constants.Arm.SAFE_BARGE_DISTANCE
        || drive.poseEstimator.getEstimatedPosition().getX() > Constants.Field.FIELD_X_SIZE / 2
            + Constants.Arm.SAFE_BARGE_DISTANCE;
  }

  public boolean atSafeProcessorDistance() {
    return drive.poseEstimator.getEstimatedPosition().getY() > Constants.Field.SAFE_WALL_DISTANCE &&
        drive.poseEstimator.getEstimatedPosition()
            .getY() < (Constants.Field.FIELD_Y_SIZE - Constants.Field.SAFE_WALL_DISTANCE);
  }

  public double positionFromAngle(double angle, boolean respectReef) {
    List<Double> positions = Arrays.asList(Utils.wrapTo0_2PI(angle), Utils.wrapTo0_2PI(angle) - 2 * Math.PI)
        .stream()
        .filter(p -> p >= Constants.Arm.ALLOWED_OPERATING_RANGE_MIN && p <= Constants.Arm.ALLOWED_OPERATING_RANGE_MAX)
        .collect(Collectors.toList());

    final double actualArmPosition = getPosition();
    final Side closeSide = getSideCloserToReef();

    double p;
    if (positions.size() == 1) {
      p = positions.get(0);
    } else if (respectReef) {
      switch (closeSide) {
        case Neither:
          // Choose whichever position is closest to actualArmPosition
          p = positions.stream()
              .min((p1, p2) -> Double.compare(Math.abs(p1 - actualArmPosition), Math.abs(p2 - actualArmPosition)))
              .orElse(0.0);
          break;
        case Right:
          // If moving from 'up' to 'down' on the right side, choose the negative angle
          // (rotate out left) if the arm is currently 'up'
          p = actualArmPosition < Math.PI / 2 ? positions.get(1) : positions.get(0);
          break;
        case Left:
          // If moving from 'up' to 'down' on the left side, choose the positive angle
          // (rotate out right) if the arm is currently 'up'
          p = actualArmPosition > -Math.PI / 2 ? positions.get(0) : positions.get(1);
          break;
        default:
          p = positions.stream()
              .min((p1, p2) -> Double.compare(Math.abs(p1 - actualArmPosition), Math.abs(p2 - actualArmPosition)))
              .orElse(0.0);
      }
    } else {
      p = positions.stream()
          .min((p1, p2) -> Double.compare(Math.abs(p1 - actualArmPosition), Math.abs(p2 - actualArmPosition)))
          .orElse(0.0);
    }

    // Keep arm inside robot if the setpoint requires pivoting towards the reef
    final boolean notAtSafeReefDistance = !atSafeReefDistance();
    if (actualArmPosition > Math.PI / 2 && p < Math.PI / 2 && closeSide == Side.Right && notAtSafeReefDistance) {
      isArmStuck = true;
      p = Math.max(p, Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
    } else if (actualArmPosition < -Math.PI / 2 && p > -Math.PI / 2 && closeSide == Side.Left
        && notAtSafeReefDistance) {
      isArmStuck = true;
      p = Math.min(p, -Math.PI + Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
    } else {
      isArmStuck = false;
    }

    // CLAMPING, DO NOT MOVE OR REMOVE!
    final double actualElevatorHeight = rC.subsystems.elevator.getHeight();

    final InterpolatingDoubleTreeMap currentMap = (rC.subsystems.intake
        .getEffectivePivotState() == Constants.Intake.PivotState.Down
        && rC.subsystems.intake.atSetpoint())
            ? elevatorToArmWhenIntakeDown
            : elevatorToArm;

    // This value is actually measured in radians DOWN from straight-up
    final double limit = currentMap.get(actualElevatorHeight);

    if (MathUtil.isNear(Math.PI, limit, 0.0001)) {
      return p; // mitigates IEEE754 issues
    } else if (actualArmPosition < 0.0) {
      // Negative arm angles: clamp between -PI - limit and -PI + limit
      return MathUtil.clamp(p, -Math.PI - limit, -Math.PI + limit);
    } else {
      // Positive arm angles: clamp between PI - limit and PI + limit
      return MathUtil.clamp(p, Math.PI - limit, Math.PI + limit);
    }
  }

  public double getDesiredPosition() {
    if (lastUpdatedTick == rC.tickNumber) {
      return lastCachedValue;
    }

    long milliseconds = System.nanoTime();
    double answer = positionFromAngle(pivotState.getDesiredAngle(this),
        pivotState.mirrorType != MirrorType.ActuallyFixedAngle);
    // milliseconds = TimeUnit.NANOSECONDS.toMillis(System.nanoTime() -
    // milliseconds);

    // lastUpdatedTick = Robot.tickNumber;
    lastCachedValue = answer;

    // if (milliseconds > 5) {
    // System.out.println("desired position took " + milliseconds + " ms");
    // }
    return answer;
  }

  // Hareket Kontrol Metotları
  // --------------------------------------------------------------------------------

  public void setState(PivotState pivot, RollerState rollers) {
    pivotState = pivot;
    rollerState = rollers;
  }

  public void setCoastEnabled(boolean coast) {
    if (coast) {
      armPivotMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      armPivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  // For when the arm skips.
  public void offsetArm(double r) {
    armOffsetRadians += r;
  }

  public double closeClampedPosition() {
    double x = absoluteEncoder.getPosition().getValueAsDouble()
        - Constants.Arm.PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS;
    while (x < -0.5)
      x += 1.0;
    while (x > 0.5)
      x -= 1.0;
    final double rawReadingArmRotations = x * Constants.Arm.PIVOT_ENCODER_RATIO;

    final double allowedOffsetArmRotations = 12.0 / 360.0;

    // Hack
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
    isZeroed = true;
  }

  // Periyodik Metot
  // --------------------------------------------------------------------------------

  @Override
  public void periodic() {
    if (Controls.wantOffsetArmPositive)
      offsetArm(armOffsetIncrementRadians);
    if (Controls.wantOffsetArmNegative)
      offsetArm(-armOffsetIncrementRadians);

    final boolean atStartOfAuto = (rC.robot.isAutonomous() && autoTimer.get() < 0.75);
    statorCurrentSignal.refresh();

    final boolean undebouncedHasObject = (rollerState == RollerState.Idle || rollerState == RollerState.SlowIdle)
        ? statorCurrentSignal.getValueAsDouble() > Constants.Arm.IDLE_CURRENT_DRAW
        : statorCurrentSignal.getValueAsDouble() > Constants.Arm.CURRENT_DRAW;

    final boolean debouncedHasCoral = coralCurrentDebouncer.calculate(undebouncedHasObject);
    final boolean debouncedHasAlgae = algaeCurrentDebouncer.calculate(undebouncedHasObject);

    hasObject = atStartOfAuto || (rollerState == RollerState.AlgaeIdle ? debouncedHasAlgae : debouncedHasCoral);

    if (!isZeroed || !rC.subsystems.elevator.isZeroed()) {
      return;
    }

    rollerMotor.set(
        atStartOfAuto ? RollerState.FastIdle.dutyCycle : rollerState.dutyCycle);

    final double positionSetpoint = getDesiredPosition();
    // Konum radyan cinsindendir, sin(position) yerçekimi beslemesini hesaplar.
    final double gravityFeedforward = Constants.Arm.POSITION_DEPENDENT_KG * Math.sin(getPosition());

    // MotionMagicVoltage kontrolü motor dönüşü cinsinden pozisyon alır
    armPivotMotor.setControl(new MotionMagicVoltage(
        (positionSetpoint - armOffsetRadians) / (2 * Math.PI)).withFeedForward(gravityFeedforward));
  }

  // SendableBuilder
  // --------------------------------------------------------------------------------

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Arm offset (deg)", () -> Math.toDegrees(armOffsetRadians), null);
    builder.addDoubleProperty("Arm position (deg)", () -> Math.toDegrees(getPosition()), null);
    builder.addDoubleProperty("Raw Encoder", () -> absoluteEncoder.getPosition().getValueAsDouble(), null);
    builder.addBooleanProperty("Has object?", () -> hasObject, null);
    builder.addDoubleProperty("Desired position (deg)", () -> Math.toDegrees(getDesiredPosition()), null);
    builder.addDoubleProperty("Motion magic setpoint (deg)",
        () -> 360.0 * armPivotMotor.getClosedLoopReference().getValueAsDouble(), null);
    builder.addBooleanProperty("At setpoint?", () -> getAtSetpoint(), null);
    builder.addDoubleProperty("roller current", () -> statorCurrentSignal.getValueAsDouble(), null);
    Utils.addClosedLoopProperties("Arm pivot", armPivotMotor, builder);
  }

}
