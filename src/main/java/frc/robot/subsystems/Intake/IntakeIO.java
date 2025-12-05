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

public interface IntakeIO {
  public void periodic();

  public void setState(PivotState p, RollerState r);

  public boolean hasCoral();

  public boolean atSetpoint();

  public double velocity();

  public boolean unsafeToGoUp();

  public boolean isZeroed();

  public void setZeroed(boolean z);

  public void setZeroingVoltage();

  public void zero();

  public void stop();

  public RollerState getRollerState();

  public PivotState getPivotState();
}
