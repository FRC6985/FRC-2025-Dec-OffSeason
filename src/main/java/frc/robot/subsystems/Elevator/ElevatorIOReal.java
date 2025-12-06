package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.*;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Elevator.AlgaeHeight;
import frc.robot.Constants.Elevator.State;
import frc.robot.Constants.Field;
import frc.robot.RobotContainer;
import frc.robot.util.Utils;

public class ElevatorIOReal implements ElevatorIO {

  public final RobotContainer rC;

  public final TalonFX mainMotor = new TalonFX(CanIds.ElevatorMainMotor, "canivore");
  public final TalonFX followerMotor = new TalonFX(CanIds.ElevatorFollowerMotor, "canivore");

  public State state = State.Down;
  private boolean zeroed = false;

  private final InterpolatingDoubleTreeMap armToElevator = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap armToElevatorWhenIntakeDown = new InterpolatingDoubleTreeMap();
  private double lastClampedSetpointForLogging = 0.0;

  public ElevatorIOReal(RobotContainer rC) {
    this.rC = rC;
    mainMotor.getConfigurator().apply(Elevator.motorConfig);
    followerMotor.setControl(new Follower(mainMotor.getDeviceID(), true));

    // Initialize interpolation maps
    for (double[] pair : Constants.ARM_ELEVATOR_PAIRS) {
      armToElevator.put(pair[0], pair[1] + edu.wpi.first.math.util.Units.inchesToMeters(0.5));
    }
    for (double[] pair : Constants.ARM_INTERPOLATION_INTAKE_DOWN) {
      armToElevatorWhenIntakeDown.put(pair[0], pair[1] + edu.wpi.first.math.util.Units.inchesToMeters(0.5));
    }
  }

  public void setState(State _state) {
    state = _state;
  }

  public State getState() {
    return state;
  }

  public double getExtension(State s) {
    if (s == State.AutoAlgae) {
      return s.rawExtension + preferredAlgaeHeight().offset;
    } else {
      return s.rawExtension;
    }
  }

  public double getHeight() {
    return mainMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return mainMotor.getVelocity().getValueAsDouble();
  }

  public boolean atSetpoint() {
    return Math.abs(getHeight() - getExtension(state)) < Elevator.SETPOINT_THRESHOLD;
  }

  public boolean lazierAtSetpoint() {
    return Math.abs(getHeight() - getExtension(state)) < Elevator.LAZIER_SETPOINT_THRESHOLD;
  }

  public boolean atOrAboveSetpoint() {
    return (getHeight() + Elevator.SETPOINT_THRESHOLD) >= getExtension(state);
  }

  public double clampSetpoint(double s) {
    long start = System.currentTimeMillis();
    double ret;

    double armDesiredSignum = Math.signum(rC.subsystems.arm.getDesiredPosition());
    double armPosition = rC.subsystems.arm.getPosition();
    double armDesiredPosition = rC.subsystems.arm.getDesiredPosition();

    double interpolationInput;
    if (Math.signum(armPosition) != armDesiredSignum) {
      interpolationInput = Math.PI - Math.abs(MathUtil.angleModulus(0.0));
    } else if ((armPosition < 0.0 && armDesiredPosition > armPosition)
        || (armPosition > 0.0 && armDesiredPosition < armPosition)) {
      interpolationInput = Math.PI - Math.abs(MathUtil.angleModulus(armDesiredPosition));
    } else {
      interpolationInput = Math.PI - Math.abs(MathUtil.angleModulus(armPosition));
    }

    double tableValue;
    if (rC.subsystems.intake.getPivotState() == Constants.Intake.PivotState.Down
        && rC.subsystems.intake.atSetpoint()) {
      tableValue = armToElevatorWhenIntakeDown.get(interpolationInput);
    } else {
      tableValue = armToElevator.get(interpolationInput);
    }

    // Clamp between 0 and MAX_EXTENSION
    ret = Math.min(Math.max(s, 0.0), Elevator.MAX_EXTENSION);
    ret = Math.min(ret, tableValue);

    lastClampedSetpointForLogging = ret;

    long elapsed = System.currentTimeMillis() - start;
    if (elapsed > 5) {
      System.out.println("Elevator.clampSetpoint() took " + elapsed + " ms");
    }

    return ret;
  }

  public double getLastClampedSetpointForLogging() {
    return lastClampedSetpointForLogging;
  }

  public double getStatorCurrent() {
    return mainMotor.getStatorCurrent().getValueAsDouble();
  }

  public void zero() {
    mainMotor.setPosition(0.0);
    zeroed = true;
  }

  public boolean isZeroed() {
    return zeroed;
  }

  public void setZeroed(boolean z) {
    zeroed = z;
  }

  public void setZeroingVoltage() {
    mainMotor.setVoltage(Elevator.ZERO_VOLTAGE);
  }

  public void stop() {
    mainMotor.setVoltage(0.0);
  }

  public Translation2d endOfManipulatorPose() {
    Translation2d base = new Translation2d(Constants.Arm.CORAL_CENTER_OFFSET, 0.0);
    Pose2d estimatedPose = rC.subsystems.drive.poseEstimator.getEstimatedPosition();
    return base.rotateBy(estimatedPose.getRotation()).plus(estimatedPose.getTranslation());
  }

  public AlgaeHeight preferredAlgaeHeight() {
    Translation2d manipulatorPose = endOfManipulatorPose();
    Translation2d reefCenter = Utils.mirrorIfRed(Field.BLUE_REEF_CENTER, rC);
    Translation2d vector = manipulatorPose.minus(reefCenter);

    double degreesAroundReefCenter = vector.getAngle().getDegrees();

    if (rC.isRedAlliance()) {
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

  public void periodic() {
    double clampedSetpoint = clampSetpoint(getExtension(state));
    mainMotor.setControl(new MotionMagicVoltage(clampedSetpoint));
  }
}
