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

  @Override
  public InterpolatingDoubleTreeMap getElevatorToArm() {
    return io.getElevatorToArm();
  }

  @Override
  public boolean atSafeBargeDistance() {
    return io.atSafeBargeDistance();
  }

  @Override
  public boolean atSafePlacementDistance() {
    return io.atSafePlacementDistance();
  }

  @Override
  public boolean atSafeProcessorDistance() {
    return io.atSafeProcessorDistance();
  }

  @Override
  public boolean atSafeReefDistance() {
    return io.atSafeReefDistance();
  }

  @Override
  public double closeClampedPosition() {
    return io.closeClampedPosition();
  }

  @Override
  public double getArmOffsetRadians() {
    return io.getArmOffsetRadians();
  }

  @Override
  public boolean getAtSetpoint() {
    return io.getAtSetpoint();
  }

  @Override
  public double getDesiredPosition() {
    return io.getDesiredPosition();
  }

  @Override
  public boolean getHasObject() {
    return io.getHasObject();
  }

  @Override
  public PivotState getPivotState() {
    return io.getPivotState();
  }

  @Override
  public double getPosition() {
    return io.getPosition();
  }

  @Override
  public RollerState getRollerState() {
    return io.getRollerState();
  }

  @Override
  public Side getSideCloserToBarge() {
    return io.getSideCloserToBarge();
  }

  @Override
  public Side getSideCloserToProcessor() {
    return io.getSideCloserToProcessor();
  }

  @Override
  public Side getSideCloserToReef() {
    return io.getSideCloserToReef();
  }

  @Override
  public boolean isArmStuck() {
    return io.isArmStuck();
  }

  @Override
  public boolean isInsideFrame() {
    return io.isInsideFrame();
  }

  @Override
  public boolean isZeroed() {
    return io.isZeroed();
  }

  @Override
  public void offsetArm(double r) {
    io.offsetArm(r);

  }

  @Override
  public void periodic() {
    io.periodic();

  }

  @Override
  public double positionFromAngle(double angle, boolean respectReef) {
    return io.positionFromAngle(angle, respectReef);
  }

  @Override
  public void resetRelativeFromAbsolute() {
    io.resetRelativeFromAbsolute();

  }

  @Override
  public void setCoastEnabled(boolean coast) {
    io.setCoastEnabled(coast);

  }

  @Override
  public void setState(PivotState pivot, RollerState rollers) {
    io.setState(pivot, rollers);

  }

}