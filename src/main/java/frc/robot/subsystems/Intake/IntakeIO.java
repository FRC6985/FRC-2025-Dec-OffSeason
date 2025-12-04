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
  void periodic();

  void setState(PivotState p, RollerState r);

  boolean hasCoral();
  boolean pivotAtSetpoint();

  boolean unsafeToGoUp();

  public RollerState getRollerState();

  public PivotState getPivotState();
}
