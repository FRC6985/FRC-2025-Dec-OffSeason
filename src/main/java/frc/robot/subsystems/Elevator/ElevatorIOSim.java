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
package frc.robot.subsystems.Elevator;

import frc.robot.Constants.Elevator.State;

public class ElevatorIOSim implements ElevatorIO {
  @Override
  public void periodic() {
    // TODO Auto-generated method stub

  }

  @Override
  public boolean atSetpoint() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public State getState() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void setState(State state) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getStatorCurrent() {
    // TODO Auto-generated method stub
    return 0;
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
