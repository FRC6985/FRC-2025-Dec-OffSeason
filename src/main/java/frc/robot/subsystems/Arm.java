package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.util.Utils;

public class Arm extends SubsystemBase {
  private static final Arm INSTANCE = new Arm();

  public static Arm getInstance() {
    return INSTANCE;
  }

  // Enums
  public enum RollerState {
    Off(0.0),
    SlowIdle(-0.035),
    FastIdle(-0.1),
    Idle(-0.035),
    AlgaeIdle(-0.225),
    In(-1.0),
    SlowOut(0.075),
    Out(1.0),
    Descore(0.8);

    public final double dutyCycle;

    RollerState(double dutyCycle) {
      this.dutyCycle = dutyCycle;
    }
  }

  public enum MirrorType {
    FixedAngle,
    ActuallyFixedAngle,
    ClosestToReef,
    ClosestToPosition,
    AlgaeScore,
    ProcessorScore
  }

  public enum PivotState {
    Up(Math.PI, MirrorType.FixedAngle),
    AlgaeUp(Math.PI, MirrorType.FixedAngle),
    Down(0.0, MirrorType.ActuallyFixedAngle),
    ScoreCoral(Math.toRadians(130.0), MirrorType.ClosestToReef),
    FinishScoreCoral(Math.toRadians(105.0), MirrorType.ClosestToReef),
    AboveScoreCoral(Math.toRadians(160.0), MirrorType.ClosestToReef),
    L4ScoreCoral(Math.toRadians(135.0), MirrorType.ClosestToReef),
    L4FinishScoreCoral(Math.toRadians(100.0), MirrorType.ClosestToReef),
    GetAlgae(Math.toRadians(100.0), MirrorType.ClosestToReef),
    PostAlgae(Math.toRadians(110.0), MirrorType.ClosestToReef),
    DescoreAlgae(Math.toRadians(110.0), MirrorType.ClosestToReef),
    SafeInsideRobotAngle(Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE, MirrorType.ClosestToReef),
    PreBarge(Math.toRadians(160.0), MirrorType.AlgaeScore),
    BargeScore(Math.toRadians(160.0), MirrorType.AlgaeScore),
    Processor(Math.toRadians(70.0), MirrorType.ProcessorScore),
    AlgaeGroundPickup(Math.toRadians(-78.0), MirrorType.ActuallyFixedAngle),
    ExitAlgaeGroundPickup(Math.toRadians(-95.0), MirrorType.ActuallyFixedAngle),
    PopsiclePickup(Math.toRadians(-80.0), MirrorType.ActuallyFixedAngle);

    private final double rawAngle;
    public final MirrorType mirrorType;

    PivotState(double rawAngle, MirrorType mirrorType) {
      this.rawAngle = rawAngle;
      this.mirrorType = mirrorType;
    }

    public double getDesiredAngle() {
      switch (mirrorType) {
        case ActuallyFixedAngle:
        case FixedAngle:
          return rawAngle;
        case ClosestToPosition:
          return INSTANCE.getPosition() > 0.0 ? rawAngle : -rawAngle;
        case ClosestToReef:
          switch (INSTANCE.getSideCloserToReef()) {
            case Left:
              return -rawAngle;
            case Right:
              return rawAngle;
            case Neither:
              return Math.abs(rawAngle) < Math.PI / 2 ? 0.0 : Math.PI;
          }
          break;
        case AlgaeScore:
          switch (INSTANCE.getSideCloserToBarge()) {
            case Left:
              return -rawAngle;
            case Right:
              return rawAngle;
            case Neither:
              return Math.PI;
          }
          break;
        case ProcessorScore:
          switch (INSTANCE.getSideCloserToProcessor()) {
            case Left:
              return -rawAngle;
            case Right:
              return rawAngle;
            case Neither:
              return Math.PI;
          }
          break;
      }
      return rawAngle;
    }
  }

  public enum Side {
    Left,
    Right,
    Neither
  }

  // Hardware
  private final DutyCycleEncoder absoluteEncoder;
  public final TalonFX armPivotMotor;
  public final TalonFX rollerMotor;
  private final StatusSignal<Current> statorCurrentSignal;

  // State variables
  private PivotState pivotState = PivotState.Up;
  private RollerState rollerState = RollerState.Off;
  public boolean isZeroed = false;
  public boolean isArmStuck = false;
  public boolean hasObject = false;

  // Configuration
  public final double deadzoneAngle = Math.toRadians(20.0);
  public final double armOffsetIncrementRadians = Math.toRadians(0.1);
  public double armOffsetRadians = 0.0;

  // Timing and caching
  private long lastUpdatedTick = -1;
  private double lastCachedValue = 0.0;
  private final Timer autoTimer = new Timer();

  // Debouncers
  private final Debouncer coralCurrentDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kBoth);
  private final Debouncer algaeCurrentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kBoth);

  // Interpolation maps
  private final InterpolatingDoubleTreeMap elevatorToArm;
  private final InterpolatingDoubleTreeMap elevatorToArmWhenIntakeDown;

  // Constructor
  private Arm() {
    absoluteEncoder = new DutyCycleEncoder(Constants.DioIds.ARM_ENCODER);
    armPivotMotor = new TalonFX(Constants.CanIds.ARM_PIVOT_MOTOR);
    armPivotMotor.getConfigurator().apply(Constants.Arm.PIVOT_CONFIG);

    rollerMotor = new TalonFX(Constants.CanIds.ARM_ROLLER_MOTOR);
    rollerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(80.0));

    statorCurrentSignal = rollerMotor.getStatorCurrent();
    statorCurrentSignal.setUpdateFrequency(100.0);

    // Initialize interpolation maps
    elevatorToArm = new InterpolatingDoubleTreeMap();
    for (var pair : Constants.armElevatorPairs) {
      elevatorToArm.put(pair[1], pair[0]);
    }

    elevatorToArmWhenIntakeDown = new InterpolatingDoubleTreeMap();
    for (var pair : Constants.armInterpolationIntakeDown) {
      elevatorToArmWhenIntakeDown.put(pair[1], pair[0]);
    }
  }

  public InterpolatingDoubleTreeMap getElevatorToArm() {
    return elevatorToArm;
  }

  // Getters for Side
  public Side getSideCloserToReef() {
    Rotation2d directionTowardReefCenter = Utils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER)
        .minus(Robot.getInstance().getEstimatedPose().getTranslation())
        .getAngle();
    Rotation2d directionTowardRight = Robot.getInstance().getEstimatedPose().getRotation()
        .rotateBy(Rotation2d.kCW_90deg);

    double ang = angleBetween(directionTowardReefCenter, directionTowardRight);

    if (Math.PI / 2 - deadzoneAngle < ang && ang < Math.PI / 2 + deadzoneAngle) {
      return Side.Neither;
    } else if (ang < Math.PI / 2) {
      return Side.Right;
    } else {
      return Side.Left;
    }
  }

  public Side getSideCloserToBarge() {
    boolean isOnBlue = !Robot.getInstance().isOnRedSide();
    double rotation = Robot.getInstance().getEstimatedPose().getRotation().getRadians();

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
    boolean isOnBlue = !Robot.getInstance().isOnRedSide();
    double rotation = Robot.getInstance().getEstimatedPose().getRotation().getRadians();

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

  // Distance check methods
  public boolean atSafeReefDistance() {
    return Robot.getInstance()
        .getEstimatedPose()
        .getTranslation()
        .getDistance(
            Utils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER)) > Constants.Arm.SAFE_DISTANCE_FROM_REEF_CENTER;
  }

  public boolean atSafePlacementDistance() {
    return Robot.getInstance()
        .getEstimatedPose()
        .getTranslation()
        .getDistance(Utils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER)) > Constants.Arm.SAFE_PLACEMENT_DISTANCE;
  }

  public boolean atSafeBargeDistance() {
    double x = Robot.getInstance().getEstimatedPose().getX();
    return x < Constants.Field.FIELD_X_SIZE / 2 - Constants.Arm.SAFE_BARGE_DISTANCE
        || x > Constants.Field.FIELD_X_SIZE / 2 + Constants.Arm.SAFE_BARGE_DISTANCE;
  }

  public boolean atSafeProcessorDistance() {
    double y = Robot.getInstance().getEstimatedPose().getY();
    return y > Constants.Field.SAFE_WALL_DISTANCE
        && y < (Constants.Field.FIELD_Y_SIZE - Constants.Field.SAFE_WALL_DISTANCE);
  }

  public boolean isInsideFrame() {
    double pos = MathUtil.angleModulus(getPosition());
    return Math.abs(pos) < Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE
        || Math.abs(pos) > (Math.PI - Constants.Arm.SAFE_INSIDE_ROBOT_ANGLE);
  }

  // Position calculation
  public double positionFromAngle(double angle, boolean respectReef) {
    double wrapped = Utils.wrapTo0_2PI(angle);
    java.util.List<Double> positions = new java.util.ArrayList<>();
    positions.add(wrapped);
    double alternative = wrapped - 2 * Math.PI;
    if (alternative >= Constants.Arm.ALLOWED_OPERATING_RANGE_MIN
        && alternative <= Constants.Arm.ALLOWED_OPERATING_RANGE_MAX) {
      positions.add(alternative);
    }

    double actualArmPosition = getPosition();
    Side closeSide = getSideCloserToReef();

    double p;
    if (positions.size() == 1) {
      p = positions.get(0);
    } else if (respectReef) {
      switch (closeSide) {
        case Neither:
          p = positions.stream()
              .min(
                  (a, b) -> Double.compare(
                      Math.abs(a - actualArmPosition), Math.abs(b - actualArmPosition)))
              .orElse(positions.get(0));
          break;
        case Right:
          p = actualArmPosition < Math.PI / 2 ? positions.get(1) : positions.get(0);
          break;
        case Left:
          p = actualArmPosition > -Math.PI / 2 ? positions.get(0) : positions.get(1);
          break;
        default:
          p = positions.get(0);
      }
    } else {
      p = positions.stream()
          .min(
              (a, b) -> Double.compare(
                  Math.abs(a - actualArmPosition), Math.abs(b - actualArmPosition)))
          .orElse(positions.get(0));
    }

    // Keep arm inside robot
    boolean notAtSafeReefDistance = !atSafeReefDistance();
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

    // Clamping
    double actualElevatorHeight = Elevator.getInstance().getHeight();
    double limit = (Intake.getInstance().getEffectivePivotState() == Intake.PivotState.Down
        && Intake.getInstance().isAtSetpoint())
            ? elevatorToArmWhenIntakeDown.get(actualElevatorHeight)
            : elevatorToArm.get(actualElevatorHeight);

    if (MathUtil.isNear(Math.PI, limit, 0.0001)) {
      return p;
    } else if (actualArmPosition < 0.0) {
      return Math.max(Math.min(p, -Math.PI + limit), -Math.PI - limit);
    } else {
      return Math.max(Math.min(p, Math.PI + limit), Math.PI - limit);
    }
  }

  public double getDesiredPosition() {
    if (lastUpdatedTick == Robot.getInstance().getTickNumber()) {
      return lastCachedValue;
    }

    long startTime = System.nanoTime();
    double answer = positionFromAngle(
        pivotState.getDesiredAngle(), pivotState.mirrorType != MirrorType.ActuallyFixedAngle);
    long milliseconds = (System.nanoTime() - startTime) / 1_000_000;

    lastUpdatedTick = Robot.getInstance().getTickNumber();
    lastCachedValue = answer;

    if (milliseconds > 5) {
      System.out.println("desired position took " + milliseconds + " ms");
    }

    return answer;
  }

  // Hardware control methods
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

  public void offsetArm(double r) {
    armOffsetRadians += r;
  }

  public void resetRelativeFromAbsolute() {
    armPivotMotor.setPosition(closeClampedPosition());
    isZeroed = true;
  }

  // Utility methods
  private double closeClampedPosition() {
    double rawPosition = absoluteEncoder.get(); // 0.0 to 1.0
    double x = rawPosition - Constants.Arm.PIVOT_ABS_ENCODER_OFFSET_ENCODER_ROTATIONS;
    while (x < -0.5)
      x += 1.0;
    while (x > 0.5)
      x -= 1.0;

    double rawReadingArmRotations = x * Constants.Arm.PIVOT_ENCODER_RATIO;
    double allowedOffsetArmRotations = 12.0 / 360.0;

    if (Math.abs(rawReadingArmRotations - 0.5) < allowedOffsetArmRotations) {
      return 0.5;
    } else if (Math.abs(rawReadingArmRotations - (-0.5)) < allowedOffsetArmRotations) {
      return -0.5;
    } else {
      return rawReadingArmRotations;
    }
  }

  private double angleBetween(Rotation2d r1, Rotation2d r2) {
    return Math.acos(r1.getCos() * r2.getCos() + r1.getSin() * r2.getSin());
  }

  // Getters
  public double getPosition() {
    return armPivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI + armOffsetRadians;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getDesiredPosition() - getPosition()) < Constants.Arm.SETPOINT_THRESHOLD;
  }

  public boolean getUndebouncedHasObject() {
    double threshold = (rollerState == RollerState.Idle || rollerState == RollerState.SlowIdle)
        ? Constants.Arm.IDLE_CURRENT_DRAW
        : Constants.Arm.CURRENT_DRAW;
    return statorCurrentSignal.getValueAsDouble() > threshold;
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  public RollerState getRollerState() {
    return rollerState;
  }

  // Lifecycle
  @Override
  public void periodic() {
    if (Controls.getInstance().wantOffsetArmPositive())
      offsetArm(armOffsetIncrementRadians);
    if (Controls.getInstance().wantOffsetArmNegative())
      offsetArm(-armOffsetIncrementRadians);

    boolean atStartOfAuto = Robot.getInstance().isAutonomous() && autoTimer.get() < 0.75;
    statorCurrentSignal.refresh();

    boolean debouncedHasCoral = coralCurrentDebouncer.calculate(getUndebouncedHasObject());
    boolean debouncedHasAlgae = algaeCurrentDebouncer.calculate(getUndebouncedHasObject());
    hasObject = atStartOfAuto
        || (rollerState == RollerState.AlgaeIdle ? debouncedHasAlgae : debouncedHasCoral);

    if (!isZeroed || !Elevator.getInstance().isZeroed()) {
      return;
    }

    double rollerDutyCycle = atStartOfAuto ? RollerState.FastIdle.dutyCycle : rollerState.dutyCycle;
    rollerMotor.set(rollerDutyCycle);

    double positionSetpoint = getDesiredPosition();
    double gravityFeedforward = Constants.Arm.POSITION_DEPENDENT_KG * Math.sin(getPosition());

    armPivotMotor.setControl(
        new MotionMagicVoltage((positionSetpoint - armOffsetRadians) / (2 * Math.PI))
            .withFeedForward(gravityFeedforward));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Arm offset (deg)", () -> Math.toDegrees(armOffsetRadians), null);
    builder.addDoubleProperty("Arm position (deg)", () -> Math.toDegrees(getPosition()), null);
    builder.addDoubleProperty(
        "Raw Encoder", () -> absoluteEncoder.get(), null);
    builder.addBooleanProperty("Has object? ", () -> hasObject, null);
    builder.addDoubleProperty(
        "Desired position (deg)", () -> Math.toDegrees(getDesiredPosition()), null);
    builder.addDoubleProperty(
        "Motion magic setpoint (deg)",
        () -> 360.0 * armPivotMotor.getClosedLoopReference().getValueAsDouble(),
        null);
    builder.addBooleanProperty("At setpoint?", () -> isAtSetpoint(), null);
    builder.addDoubleProperty("roller current", () -> statorCurrentSignal.getValueAsDouble(), null);
    Utils.addClosedLoopProperties("Arm pivot", armPivotMotor, builder);
  }
}
