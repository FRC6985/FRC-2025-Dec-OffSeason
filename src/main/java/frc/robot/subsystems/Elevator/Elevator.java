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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator.State;

public class Elevator extends SubsystemBase implements ElevatorIO {
  ElevatorIO io;

  public Elevator() {
  }

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void setState(State state) {
    io.setState(state);
  }

  @Override
  public void periodic() {
    io.periodic();
  }

  @Override
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  @Override
  public State getState() {
    return io.getState();
  }

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
  public void stop() {
    io.stop();

  }

  @Override
  public void zero() {
    io.zero();
  }

  @Override
  public double getStatorCurrent() {
    return io.getStatorCurrent();
  }
}
