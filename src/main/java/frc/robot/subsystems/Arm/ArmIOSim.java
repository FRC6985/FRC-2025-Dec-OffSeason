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

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.Arm.PivotState;
import frc.robot.Constants.Arm.RollerState;
import frc.robot.Constants.Arm.Side;

public class ArmIOSim implements ArmIO {

  @Override
  public boolean atSafeReefDistance() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public boolean atSetpoint() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public double closeClampedPosition() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getDesiredPosition() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean getInsideFrame() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public double getPosition() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public Side getSideCloserToBarge() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Side getSideCloserToProcessor() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Side getSideCloserToReef() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public boolean getUndebouncedHasObject() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void offsetArm(double r) {
    // TODO Auto-generated method stub

  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub

  }

  @Override
  public double positionFromAngle(double angle, boolean respectReef) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void resetRelativeFromAbsolute() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setCoastEnabled(boolean coast) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setState(PivotState pivot, RollerState rollers) {
    // TODO Auto-generated method stub

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub

  }

  @Override
  public boolean isZeroed() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void setZeroed(boolean z) {
    // TODO Auto-generated method stub

  }
}
