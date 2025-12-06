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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm.PivotState;
import frc.robot.Constants.Arm.RollerState;
import frc.robot.Constants.Arm.Side;

public class Arm extends SubsystemBase implements ArmIO {
  ArmIO io;

  public Arm() {
  }

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void setState(PivotState p, RollerState r) {
    io.setState(p, r);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    io.initSendable(builder);
  }

  @Override
  public void periodic() {
    io.periodic();
  }

  @Override
  public Side getSideCloserToBarge() {
    return io.getSideCloserToBarge();
  }

  @Override
  public boolean atSafeReefDistance() {
    return io.atSafeReefDistance();
  }

  @Override
  public boolean getInsideFrame() {
    return io.getInsideFrame();
  }

  @Override
  public void offsetArm(double offset) {
    io.offsetArm(offset);
  }

  @Override
  public double closeClampedPosition() {
    return io.closeClampedPosition();
  }

  @Override
  public Side getSideCloserToReef() {
    return io.getSideCloserToReef();
  }

  @Override
  public double getPosition() {
    return io.getPosition();
  }

  @Override
  public void setCoastEnabled(boolean enabled) {
    io.setCoastEnabled(enabled);
  }

  @Override
  public void resetRelativeFromAbsolute() {
    io.resetRelativeFromAbsolute();
  }

  @Override
  public boolean getUndebouncedHasObject() {
    return io.getUndebouncedHasObject();
  }

  @Override
  public double getDesiredPosition() {
    return io.getDesiredPosition();
  }

  @Override
  public double positionFromAngle(double angle, boolean isAbsolute) {
    return io.positionFromAngle(angle, isAbsolute);
  }

  @Override
  public Side getSideCloserToProcessor() {
    return io.getSideCloserToProcessor();
  }

  @Override
  public boolean atSetpoint() {
    return io.atSetpoint();
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
  public InterpolatingDoubleTreeMap getElevatorToArm() {
    return io.getElevatorToArm();
  }
}
