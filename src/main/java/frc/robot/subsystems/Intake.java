package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.util.Utils;

public class Intake extends SubsystemBase {
  private static final Intake INSTANCE = new Intake();

  public static Intake getInstance() {
    return INSTANCE;
  }

  // Enums
  public enum PivotState {
    Down(Math.toRadians(126.0)),
    Trough(Math.toRadians(25.639507)),
    Up(0.0),
    OperatorControl(0.0);

    public final double angleSetpoint;

    PivotState(double angleSetpoint) {
      this.angleSetpoint = angleSetpoint;
    }
  }

  public enum RollerState {
    In(-6.0, -8.0),
    SlowIn(-2.0, -3.0),
    TroughOut(3.25, 0.0),
    Out(8.0, 0.0),
    Off(0.0, 0.0),
    AlgaeModeIdle(0.0, 0.0),
    OperatorControl(0.0, 0.0);

    public final double rollingVolts;
    public final double centeringVoltage;

    RollerState(double rollingVolts, double centeringVoltage) {
      this.rollingVolts = rollingVolts;
      this.centeringVoltage = centeringVoltage;
    }
  }

  // Hardware
  private final TalonFX pivotMotor;
  private final TalonFX rollerMotor;
  private final TalonFX centeringMotor;
  private final DigitalInput linebreak;

  // State variables
  private PivotState realPivotState = PivotState.Up;
  private RollerState realRollerState = RollerState.Off;
  public boolean isZeroed = false;

  // Constructor
  private Intake() {
    pivotMotor = new TalonFX(Constants.CanIds.INTAKE_PIVOT_MOTOR);
    pivotMotor.getConfigurator().apply(Constants.Intake.PIVOT_CONFIG);

    rollerMotor = new TalonFX(Constants.CanIds.INTAKE_ROLLER_MOTOR);
    rollerMotor
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    centeringMotor = new TalonFX(Constants.CanIds.INTAKE_CENTERING_MOTOR);
    centeringMotor
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    linebreak = new DigitalInput(Constants.DioIds.INTAKE_LINEBREAK);
  }

  // Control methods
  public void zero() {
    pivotMotor.setPosition(0.0);
    isZeroed = true;
  }

  public void setZeroingVoltage() {
    pivotMotor.setVoltage(Constants.Intake.ZERO_VOLTAGE);
  }

  public void stop() {
    pivotMotor.setVoltage(0.0);
    rollerMotor.setVoltage(0.0);
    centeringMotor.setVoltage(0.0);
  }

  public void setState(PivotState p, RollerState r) {
    realPivotState = p;

    if (hasCoral() && r == RollerState.Off) {
      if (Controls.getInstance().getSuperstructureInputs().wantedScoringLevel
          != Superstructure.ScoringLevel.TROUGH) {
        realRollerState = RollerState.In;
      } else {
        realRollerState = RollerState.SlowIn;
      }
    } else if (hasCoral() && r == RollerState.AlgaeModeIdle) {
      realRollerState = RollerState.SlowIn;
    } else {
      realRollerState = r;
    }
  }

  // Getters
  public double getAngle() {
    return pivotMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  public double getVelocity() {
    return pivotMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  public boolean hasCoral() {
    return !linebreak.get() || Controls.getInstance().operatorController.getStartButton();
  }

  public boolean isAtSetpoint() {
    return Math.abs(getAngle() - getEffectivePivotState().angleSetpoint)
        < Constants.Intake.SETPOINT_THRESHOLD;
  }

  public PivotState getEffectivePivotState() {
    if (realPivotState != PivotState.OperatorControl) {
      return realPivotState;
    } else if (isUnsafeToGoUp()) {
      return PivotState.Down;
    } else if (hasCoral()) {
      return PivotState.Up;
    } else if (Superstructure.getInstance().inputs.wantGroundIntake) {
      return PivotState.Down;
    } else {
      return PivotState.Up;
    }
  }

  public RollerState getEffectiveRollerState() {
    if (realRollerState != RollerState.OperatorControl) {
      return realRollerState;
    } else if (Superstructure.getInstance().inputs.wantGroundIntake) {
      return RollerState.In;
    } else if (hasCoral()) {
      return RollerState.SlowIn;
    } else {
      return RollerState.Off;
    }
  }

  private boolean isUnsafeToGoUp() {
    double armPosition = Math.abs(MathUtil.angleModulus(Arm.getInstance().getPosition()));
    double elevatorToArmLimit =
        Math.PI - Arm.getInstance().getElevatorToArm().get(Elevator.getInstance().getHeight());
    return armPosition < elevatorToArmLimit;
  }

  // Lifecycle
  @Override
  public void periodic() {
    if (!isZeroed) {
      return;
    }

    PivotState effectivePivot = getEffectivePivotState();
    RollerState effectiveRoller = getEffectiveRollerState();

    pivotMotor.setControl(new MotionMagicVoltage(effectivePivot.angleSetpoint / (2 * Math.PI)));
    rollerMotor.setVoltage(effectiveRoller.rollingVolts);
    centeringMotor.setVoltage(effectiveRoller.centeringVoltage);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Intake angle", () -> Math.toDegrees(getAngle()), null);
    builder.addDoubleProperty(
        "Intake setpoint", () -> Math.toDegrees(getEffectivePivotState().angleSetpoint), null);
    builder.addBooleanProperty("at setpoint?", this::isAtSetpoint, null);
    builder.addBooleanProperty("Intake have coral", this::hasCoral, null);
    builder.addStringProperty(
        "Effective intake pivot state", () -> getEffectivePivotState().toString(), null);
    builder.addStringProperty(
        "Underlying intake pivot state", () -> realPivotState.toString(), null);
    builder.addBooleanProperty("Is Zeroed?", () -> isZeroed, null);
    Utils.addClosedLoopProperties("Intake Pivot", pivotMotor, builder);
    builder.addBooleanProperty("unsafe for intake to go up?", this::isUnsafeToGoUp, null);
  }
}
