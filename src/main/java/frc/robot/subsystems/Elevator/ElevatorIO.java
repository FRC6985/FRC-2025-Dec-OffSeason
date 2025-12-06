package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Elevator.AlgaeHeight;
import frc.robot.Constants.Elevator.State;

public interface ElevatorIO {
  void periodic();

  void setState(State state);

  State getState();

  boolean atSetpoint();

  boolean lazierAtSetpoint();

  boolean atOrAboveSetpoint();

  double getExtension(State s);

  double getHeight();

  double getVelocity();

  double clampSetpoint(double s);

  double getLastClampedSetpointForLogging();

  Translation2d endOfManipulatorPose();

  AlgaeHeight preferredAlgaeHeight();

  boolean isZeroed();

  void setZeroed(boolean z);

  void setZeroingVoltage();

  void zero();

  void stop();

  double getStatorCurrent();
}
