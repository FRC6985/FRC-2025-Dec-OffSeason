package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.AlgaeHeight;
import frc.robot.Constants.Elevator.State;

public class Elevator extends SubsystemBase implements ElevatorIO {
  ElevatorIO io;

  // Constructor
  public Elevator() {

  }

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  // Periodic
  @Override
  public void periodic() {
    io.periodic();
  }

  // State control
  @Override
  public void setState(State state) {
    io.setState(state);
  }

  @Override
  public State getState() {
    return io.getState();
  }

  // Setpoint checks
  @Override
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  @Override
  public boolean lazierAtSetpoint() {
    return io.lazierAtSetpoint();
  }

  @Override
  public boolean atOrAboveSetpoint() {
    return io.atOrAboveSetpoint();
  }

  // Elevator kinematics
  @Override
  public double getExtension(State s) {
    return io.getExtension(s);
  }

  @Override
  public double getHeight() {
    return io.getHeight();
  }

  @Override
  public double getVelocity() {
    return io.getVelocity();
  }

  @Override
  public double clampSetpoint(double s) {
    return io.clampSetpoint(s);
  }

  @Override
  public double getLastClampedSetpointForLogging() {
    return io.getLastClampedSetpointForLogging();
  }

  // Manipulator pose
  @Override
  public Translation2d endOfManipulatorPose() {
    return io.endOfManipulatorPose();
  }

  @Override
  public AlgaeHeight preferredAlgaeHeight() {
    return io.preferredAlgaeHeight();
  }

  // Zeroing & control
  @Override
  public boolean isZeroed() {
    return io.isZeroed();
  }

  @Override
  public void setZeroed(boolean z) {
    io.setZeroed(z);
  }

  @Override
  public void setZeroingVoltage() {
    io.setZeroingVoltage();
  }

  @Override
  public void zero() {
    io.zero();
  }

  @Override
  public void stop() {
    io.stop();
  }

  @Override
  public double getStatorCurrent() {
    return io.getStatorCurrent();
  }
}
