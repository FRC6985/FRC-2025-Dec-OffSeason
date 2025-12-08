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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.PivotState;
import frc.robot.Constants.Intake.RollerState;

public class Intake extends SubsystemBase implements IntakeIO {
  IntakeIO io;

  public Intake() {
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public PivotState getEffectivePivotState() {
    return io.getEffectivePivotState();
  }

  @Override
  public RollerState getEffectiveRollerState() {
    return io.getEffectiveRollerState();
  }

  @Override
  public boolean hasCoral() {
    return io.hasCoral();
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
  public void setState(PivotState pivotState, RollerState rollerState) {
    io.setState(pivotState, rollerState);
  }

  @Override
  public boolean isZeroed() {
    return io.isZeroed();
  }

  @Override
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  @Override
  public void setZeroed(boolean zeroed) {
    io.setZeroed(zeroed);
  }

  @Override
  public void stop() {
    io.stop();
  }

  @Override
  public double getAngle() {
    return io.getAngle();
  }

  @Override
  public double getVelocity() {
    return io.getVelocity();
  }

  @Override
  public void periodic() {
    super.periodic();
    io.periodic();
  }
}
