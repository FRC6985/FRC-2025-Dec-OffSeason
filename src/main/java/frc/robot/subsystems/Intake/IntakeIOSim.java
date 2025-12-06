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
package frc.robot.subsystems.Intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.Intake.PivotState;
import frc.robot.Constants.Intake.RollerState;

public class IntakeIOSim implements IntakeIO {

  @Override
  public boolean atSetpoint() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public double getAngle() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVelocity() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean hasCoral() {
    // TODO Auto-generated method stub
    return false;
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
  public void periodic() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setState(PivotState p, RollerState r) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setZeroed(boolean z) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setZeroingVoltage() {
    // TODO Auto-generated method stub

  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

  @Override
  public void zero() {
    // TODO Auto-generated method stub

  }

}
