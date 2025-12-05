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

public class Intake extends SubsystemBase {
  IntakeIO io;

  public Intake() {}

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setState(PivotState p, RollerState r) {
    io.setState(p, r);
  }

  @Override
  public void periodic() {
    io.periodic();
  }
}
