package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Utils;

public class Elevator extends SubsystemBase {
  private static final Elevator INSTANCE = new Elevator();

  public static Elevator getInstance() {
    return INSTANCE;
  }

  // Enums
  public enum State {
    Down(0.0),
    PreHandoff(Units.inchesToMeters(36.0)),
    Handoff(Units.inchesToMeters(33.25)),
    SourceIntake(Units.inchesToMeters(53.0)),
    PreScore(Units.inchesToMeters(20.0)),
    Trough(Units.inchesToMeters(38.0)),
    L2(Units.inchesToMeters(15.0)),
    L3(State.L2.rawExtension + Units.inchesToMeters(15.8701)),
    L4(Units.inchesToMeters(54.5 - 0.125)),
    Barge(Units.inchesToMeters(55.0 - 0.125)),
    ScoreL4(State.L4.rawExtension - Units.inchesToMeters(1.0)),
    ScoreL3(State.L3.rawExtension - Units.inchesToMeters(3.5)),
    ScoreL2(State.L2.rawExtension - Units.inchesToMeters(3.5)),
    PostL3(State.L2.rawExtension - Units.inchesToMeters(6.0)),
    PostL2(State.L2.rawExtension - Units.inchesToMeters(3.5)),
    AutoAlgae(Units.inchesToMeters(21.75)),
    LowAlgae(Units.inchesToMeters(22.25)),
    HighAlgae(State.LowAlgae.rawExtension + Units.inchesToMeters(15.8701)),
    Processor(Units.inchesToMeters(20.0)),
    AlgaeRest(Units.inchesToMeters(15.0)),
    GroundAlgaeIntake(0.14),
    PopsiclePickup(0.065);

    public final double rawExtension;

    State(double rawExtension) {
      this.rawExtension = rawExtension;
    }

    public double getExtension() {
      if (this == AutoAlgae) {
        return rawExtension + Elevator.getInstance().getPreferredAlgaeHeight().offset;
      } else {
        return rawExtension;
      }
    }
  }

  public enum AlgaeHeight {
    High(Units.inchesToMeters(15.8701)),
    Low(0.0);

    public final double offset;

    AlgaeHeight(double offset) {
      this.offset = offset;
    }
  }

  // Hardware
  private final TalonFX mainMotor;
  private final TalonFX followerMotor;

  // State variables
  public boolean isZeroed = false;
  public State state = State.Down;
  public double lastClampedSetpointForLogging = 0.0;

  // Interpolation maps
  private final InterpolatingDoubleTreeMap armToElevator;
  private final InterpolatingDoubleTreeMap armToElevatorWhenIntakeDown;

  // Constructor
  private Elevator() {
    mainMotor = new TalonFX(Constants.CanIds.ELEVATOR_MAIN_MOTOR, "canivore");
    mainMotor.getConfigurator().apply(Constants.Elevator.MOTOR_CONFIG);

    followerMotor = new TalonFX(Constants.CanIds.ELEVATOR_FOLLOWER_MOTOR, "canivore");
    followerMotor.setControl(new Follower(mainMotor.getDeviceID(), true));

    // Initialize interpolation maps
    armToElevator = new InterpolatingDoubleTreeMap();
    for (var pair : Constants.armElevatorPairs) {
      armToElevator.put(pair[0], pair[1] + Units.inchesToMeters(0.5));
    }

    armToElevatorWhenIntakeDown = new InterpolatingDoubleTreeMap();
    for (var pair : Constants.armInterpolationIntakeDown) {
      armToElevatorWhenIntakeDown.put(pair[0], pair[1] + Units.inchesToMeters(0.5));
    }
  }

  // Control methods
  public boolean isZeroed() {
    return isZeroed;
  }

  public void zero() {
    mainMotor.setPosition(0.0);
    isZeroed = true;
  }

  public void setZeroingVoltage() {
    mainMotor.setVoltage(Constants.Elevator.ZERO_VOLTAGE);
  }

  public void stop() {
    mainMotor.setVoltage(0.0);
  }

  public void setCoastEnabled(boolean coast) {
    if (coast) {
      mainMotor.setNeutralMode(NeutralModeValue.Coast);
      followerMotor.setNeutralMode(NeutralModeValue.Coast);
    } else {
      mainMotor.setNeutralMode(NeutralModeValue.Brake);
      followerMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  // Position clamping logic
  public double clampSetpoint(double s) {
    double ret = 0.0;
    long startTime = System.currentTimeMillis();

    double armDesiredPositionSignum = Math.signum(Arm.getInstance().getDesiredPosition());
    double armPositionSignum = Math.signum(Arm.getInstance().getPosition());

    double interpolationTableInput;
    if (armPositionSignum != armDesiredPositionSignum) {
      interpolationTableInput = 0.0;
    } else if ((Arm.getInstance().getPosition() < 0.0
        && Arm.getInstance().getDesiredPosition() > Arm.getInstance().getPosition())
        || (Arm.getInstance().getPosition() > 0.0
            && Arm.getInstance().getDesiredPosition() < Arm.getInstance().getPosition())) {
      interpolationTableInput = Arm.getInstance().getDesiredPosition();
    } else {
      interpolationTableInput = Arm.getInstance().getPosition();
    }

    interpolationTableInput = Math.PI - Math.abs(MathUtil.angleModulus(interpolationTableInput));

    double minHeight = (Intake.getInstance().getEffectivePivotState() == Intake.PivotState.Down
        && Intake.getInstance().isAtSetpoint())
            ? armToElevatorWhenIntakeDown.get(interpolationTableInput)
            : armToElevator.get(interpolationTableInput);

    ret = Math.max(Math.min(s, Constants.Elevator.MAX_EXTENSION), minHeight);
    lastClampedSetpointForLogging = ret;

    long time = System.currentTimeMillis() - startTime;
    if (time > 5) {
      System.out.println("Elevator. clampSetpoint() took " + time + " ms");
    }

    return ret;
  }

  // Getters
  public double getHeight() {
    return mainMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return mainMotor.getVelocity().getValueAsDouble();
  }

  public double getStatorCurrent() {
    return mainMotor.getStatorCurrent().getValueAsDouble();
  }

  public boolean isAtSetpoint() {
    return Math.abs(getHeight() - state.getExtension()) < Constants.Elevator.SETPOINT_THRESHOLD;
  }

  public boolean isLazierAtSetpoint() {
    return Math.abs(getHeight() - state.getExtension()) < Constants.Elevator.LAZIER_SETPOINT_THRESHOLD;
  }

  public boolean isAtOrAboveSetpoint() {
    return (getHeight() + Constants.Elevator.SETPOINT_THRESHOLD) >= state.getExtension();
  }

  public AlgaeHeight getPreferredAlgaeHeight() {
    Translation2d endOfManipulatorPose = getEndOfManipulatorPose();
    Translation2d reefCenter = Utils.mirrorIfRed(Constants.Field.BLUE_REEF_CENTER);

    double degreesAroundReefCenter = endOfManipulatorPose.minus(reefCenter).getAngle().getDegrees();

    if (Robot.getInstance().isRedAlliance()) {
      degreesAroundReefCenter += 180.0;
    }

    double algaeDirection = Math.toDegrees(Utils.wrapTo0_2PI(Math.toRadians(degreesAroundReefCenter - 30.0)));

    if ((300.0 < algaeDirection && algaeDirection < 360.0)
        || (180.0 < algaeDirection && algaeDirection < 240.0)
        || (60.0 < algaeDirection && algaeDirection < 120.0)) {
      return AlgaeHeight.Low;
    } else {
      return AlgaeHeight.High;
    }
  }

  public Translation2d getEndOfManipulatorPose() {
    return new Translation2d(Constants.Arm.CORAL_CENTER_OFFSET, 0.0)
        .rotateBy(Robot.getInstance().getEstimatedPose().getRotation())
        .plus(Robot.getInstance().getEstimatedPose().getTranslation());
  }

  // Lifecycle
  @Override
  public void periodic() {
    if (!isZeroed || !Arm.getInstance().isZeroed) {
      return;
    }
    mainMotor.setControl(new MotionMagicVoltage(clampSetpoint(state.getExtension())));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Height", this::getHeight, null);
    builder.addDoubleProperty("Setpoint", () -> state.getExtension(), null);
    builder.addDoubleProperty("Clamped setpoint", () -> lastClampedSetpointForLogging, null);
    builder.addDoubleProperty(
        "Motion magic setpoint (deg)",
        () -> mainMotor.getClosedLoopReference().getValueAsDouble(),
        null);
    builder.addBooleanProperty("At setpoint?", this::isAtSetpoint, null);
    builder.addBooleanProperty("Is Zeroed?", () -> isZeroed, null);
    Utils.addClosedLoopProperties("Elevator", mainMotor, builder);
  }
}
