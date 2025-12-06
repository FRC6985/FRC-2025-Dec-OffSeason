package frc.robot.subsystems.Arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.Arm.PivotState;
import frc.robot.Constants.Arm.RollerState;
import frc.robot.Constants.Arm.Side;

public interface ArmIO {

  double getPosition();

  boolean atSetpoint();

  boolean getInsideFrame();

  boolean getUndebouncedHasObject();

  Side getSideCloserToReef();

  Side getSideCloserToBarge();

  Side getSideCloserToProcessor();

  boolean atSafeReefDistance();

  double getDesiredPosition();

  double positionFromAngle(double angle, boolean respectReef);

  void setState(PivotState pivot, RollerState rollers);

  void setCoastEnabled(boolean coast);

  void offsetArm(double r);

  double closeClampedPosition();

  void resetRelativeFromAbsolute();

  void periodic();

  void initSendable(SendableBuilder builder);
}