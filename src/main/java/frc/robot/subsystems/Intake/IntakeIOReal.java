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

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.PivotState;
import frc.robot.Constants.Intake.RollerState;

public class IntakeIOReal implements IntakeIO {
 
  public final TalonFX pivotMotor = new TalonFX(CanIds.IntakePivotMotor, "canivore");

  public final TalonFX rollerMotor = new TalonFX(CanIds.IntakeRollerMotor, "canivore");
  public final TalonFX centeringMotor = new TalonFX(CanIds.IntakeCenteringMotor, "canivore");
  public RollerState rollerState = RollerState.Off;
  public PivotState pivotState = PivotState.Up;


  public IntakeIOReal(){
    pivotMotor.getConfigurator().apply(Intake.pivotConfig);
  }

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
    if (state.equals(RollerState.In) && hasCoral()) state = RollerState.SlowIn;

    return state;
  }

  public boolean atSetpoint() {
    return Math.abs(pivotMotor.getPosition().getValueAsDouble()*2*Math.PI - getPivotState().angle)
        < Intake.SETPOINT_THRESHOLD;
  }

  public PivotState getPivotState() {
    PivotState state = pivotState;

    if (state.equals(PivotState.Down) && hasCoral()) state = PivotState.Up;

    if (state.equals(PivotState.Up) && unsafeToGoUp()) state = PivotState.Down;

    return state;
  }
  public void stop(){
    rollerMotor.setVoltage(0);
    centeringMotor.setVoltage(0);
    pivotMotor.setVoltage(0);
  }
  public void periodic() {
    rollerMotor.setVoltage(getRollerState().rollingVoltage);
    centeringMotor.setVoltage(getRollerState().centeringVoltage);
    pivotMotor.setControl(new MotionMagicVoltage(getPivotState().angle/(2*Math.PI)));
  }
}
