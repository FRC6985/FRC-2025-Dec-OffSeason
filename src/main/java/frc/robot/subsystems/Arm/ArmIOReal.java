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

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Arm.PivotState;
import frc.robot.Constants.Arm.RollerState;

public class ArmIOReal implements ArmIO {

  public final TalonFX pivotMotor = new TalonFX(CanIds.IntakePivotMotor, "canivore");
  public final MotionMagicVoltage pivotMotionControl = new MotionMagicVoltage(PivotState.Up.pos);
  // TODO: pivot motor configs will add

  public final TalonFX rollerMotor = new TalonFX(CanIds.IntakeRollerMotor, "canivore");
  public final TalonFX centeringMotor = new TalonFX(CanIds.IntakeCenteringMotor, "canivore");
  public RollerState rollerState = RollerState.Off;
  public PivotState pivotState = PivotState.Up;

  public boolean hasCoral() {
    // TODO: read sensor value
    return false;
  }

  public boolean unsafeToGoUp() {
    // TODO: calucate distance of arm and intake pos
    return false;
  }

  public void setState(PivotState p, RollerState r) {
    pivotState = p;
    rollerState = r;
  }

  public RollerState getRollerState() {
    RollerState state = rollerState;
    if (state.equals(RollerState.In) && hasCoral())
      state = RollerState.SlowIn;

    return state;
  }

  public boolean pivotAtSetpoint() {
    return Math.abs(pivotMotor.getPosition().getValueAsDouble() - getPivotState().pos) < Intake.SETPOINT_THRESHOLD;
  }

  public PivotState getPivotState() {
    PivotState state = pivotState;

    if (state.equals(PivotState.Down) && hasCoral())
      state = PivotState.Up;

    if (state.equals(PivotState.Up) && unsafeToGoUp())
      state = PivotState.Down;

    return state;
  }

  public void periodic() {
    rollerMotor.setVoltage(getRollerState().rollingVoltage);
    pivotMotor.setControl(pivotMotionControl.withPosition(getPivotState().pos));
  }
}
