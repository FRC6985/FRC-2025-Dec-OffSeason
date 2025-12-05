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

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Elevator.State;
import frc.robot.Constants.CanIds;

public class ElevatorIOReal implements ElevatorIO {

  public final TalonFX mainMotor = new TalonFX(CanIds.ElevatorMainMotor, "canivore");

  public final TalonFX followerMotor = new TalonFX(CanIds.ElevatorFollowerMotor, "canivore");
  public State state = State.Down;

  public ElevatorIOReal() {
    mainMotor.getConfigurator().apply(Elevator.motorConfig);
    followerMotor.setControl(new Follower(mainMotor.getDeviceID(), true));
  }

  public void setState(State state) {

  }

  public boolean atSetpoint() {
    // return Math.abs(mainMotor.getPosition().getValueAsDouble()*2*Math.PI -
    // getPivotState().angle)
    // < Arm.SETPOINT_THRESHOLD;
    return false;
  }

  public State getState() {
    State s = state;

    // if (state.equals(PivotState.Down) && hasCoral()) state = PivotState.Up;

    // if (state.equals(PivotState.Up) && unsafeToGoUp()) state = PivotState.Down;

    return s;
  }

  public void stop() {
    mainMotor.setVoltage(0);
  }

  public void periodic() {

    mainMotor.setControl(new MotionMagicVoltage(getState().rawExtension/* Look clampSetpoint */));
  }

}
