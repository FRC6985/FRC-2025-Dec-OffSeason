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

import frc.robot.Constants.Intake.PivotState;
import frc.robot.Constants.Intake.RollerState;

public class IntakeIOSim implements IntakeIO {

  @Override
  public boolean atSetpoint() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public PivotState getPivotState() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public RollerState getRollerState() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public boolean hasCoral() {
    // TODO Auto-generated method stub
    return false;
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
  public void setZeroingVoltage() {
    // TODO Auto-generated method stub

  }

  @Override
  public boolean unsafeToGoUp() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void zero() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setZeroed(boolean z) {
    // TODO Auto-generated method stub

  }

  @Override
  public double velocity() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

}
