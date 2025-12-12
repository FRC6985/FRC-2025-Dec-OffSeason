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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TeleopSuperstructureCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.Checkers;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  // ==================== SINGLETON ====================
  private static Robot instance;

  public static Robot getInstance() {
    return instance;
  }
  // ===================================================

  private Command autonomousCommand;
  private final LoggedDashboardChooser<Command> autoChooser;
  public final Drive drive;

  public boolean isRedAlliance() {

    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
  }

  public boolean isOnRedSide() {
    return drive.poseEstimator.getEstimatedPosition().getX() > (Constants.Field.FIELD_X_SIZE / 2);
  }

  public Pose2d getEstimatedPose() {
    return drive.poseEstimator.getEstimatedPosition();
  }

  private int tickNumber = 0;
  private boolean isAligned = false;

  public void setIsAligned(boolean aligned) {
    isAligned = aligned;
  }

  public boolean getIsAligned() {
    return isAligned;
  }

  public int getTickNumber() {
    return tickNumber;
  }

  public Robot() {
    instance = this;

    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    Checkers.checkSwerveConfigures();

    // Subsystems initialize
    Vision.getInstance();
    Arm.getInstance();
    Intake.getInstance();
    Elevator.getInstance();
    Superstructure.getInstance();

    drive = new Drive(
        new GyroIOPigeon2(),
        new ModuleIOTalonFX(TunerConstants.FrontLeft),
        new ModuleIOTalonFX(TunerConstants.FrontRight),
        new ModuleIOTalonFX(TunerConstants.BackLeft),
        new ModuleIOTalonFX(TunerConstants.BackRight));

    NamedCommands.registerCommand("intakeDown",
        Commands.runOnce(() -> {
          Superstructure.getInstance().inputs.wantGroundIntake = true;
        }));

    NamedCommands.registerCommand("intakeUp",
        Commands.runOnce(() -> {
          Superstructure.getInstance().inputs.wantGroundIntake = false;
        }));

    // Wait for coral
    NamedCommands.registerCommand("waitForCoral",
        Commands.waitUntil(() -> Intake.getInstance().hasCoral()));

    // L4 Scoring
    NamedCommands.registerCommand("prepareL4",
        Commands.runOnce(() -> {
          Superstructure.getInstance().inputs.wantExtend = true;
          Superstructure.getInstance().inputs.wantedScoringLevel = Superstructure.ScoringLevel.L4;
        }));

    NamedCommands.registerCommand("scoreL4",
        Commands.sequence(
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantScore = true;
            }),
            Commands.waitUntil(() -> Superstructure.getInstance().state == Superstructure.State.PlaceL4
                && Arm.getInstance().isAtSetpoint()
                && Elevator.getInstance().isAtSetpoint()),
            Commands.waitSeconds(0.3), // Piece release
            Commands.waitUntil(() -> !Arm.getInstance().hasObject)));

    // L3 Scoring
    NamedCommands.registerCommand("prepareL3",
        Commands.runOnce(() -> {
          Superstructure.getInstance().inputs.wantExtend = true;
          Superstructure.getInstance().inputs.wantedScoringLevel = Superstructure.ScoringLevel.L3;
        }));

    NamedCommands.registerCommand("scoreL3",
        Commands.sequence(
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantScore = true;
            }),
            Commands.waitUntil(() -> !Arm.getInstance().hasObject)));

    // L2 Scoring
    NamedCommands.registerCommand("prepareL2",
        Commands.runOnce(() -> {
          Superstructure.getInstance().inputs.wantExtend = true;
          Superstructure.getInstance().inputs.wantedScoringLevel = Superstructure.ScoringLevel.L2;
        }));

    NamedCommands.registerCommand("scoreL2",
        Commands.sequence(
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantScore = true;
            }),
            Commands.waitUntil(() -> !Arm.getInstance().hasObject)));

    // Trough
    NamedCommands.registerCommand("scoreTrough",
        Commands.sequence(
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantExtend = true;
              Superstructure.getInstance().inputs.wantedScoringLevel = Superstructure.ScoringLevel.TROUGH;
            }),
            Commands.waitUntil(() -> Intake.getInstance().isAtSetpoint()),
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantScore = true;
            }),
            Commands.waitSeconds(0.5)));

    // Algae
    NamedCommands.registerCommand("startGetAlgae",
        Commands.runOnce(() -> {
          Superstructure.getInstance().inputs.wantGetAlgae = true;
        }));

    NamedCommands.registerCommand("waitForAlgae",
        Commands.waitUntil(() -> Arm.getInstance().hasObject));

    NamedCommands.registerCommand("scoreAlgae",
        Commands.sequence(
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantExtend = true;
            }),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantScore = true;
            }),
            Commands.waitSeconds(0.5)));

    // Popsicle (vertical coral)
    NamedCommands.registerCommand("popsiclePickup",
        Commands.sequence(
            Commands.runOnce(() -> {
              Superstructure.getInstance().inputs.wantPopsiclePickup = true;
            }),
            Commands.waitSeconds(Superstructure.POPSICLE_DELAY),
            Commands.waitUntil(() -> Arm.getInstance().hasObject)));

    // Reset
    NamedCommands.registerCommand("resetInputs",
        Commands.runOnce(() -> {
          Superstructure.getInstance().emptyInputs();
        }));

    // Auto Chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  @Override
  public void robotPeriodic() {
    tickNumber++;
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    Arm.getInstance().hasObject = true;
    autonomousCommand = autoChooser.get();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    drive.setDefaultCommand(
        new TeleopDriveCommand(() -> Controls.getInstance().getDriverInputs()));

    Superstructure.getInstance().setDefaultCommand(
        new TeleopSuperstructureCommand());

    if (!Arm.getInstance().isZeroed ||
        !Elevator.getInstance().isZeroed() ||
        !Intake.getInstance().isZeroed) {
      Superstructure.getInstance().makeZeroAllSubsystemsCommand().schedule();
    }
  }

  @Override
  public void teleopExit() {
    drive.removeDefaultCommand();
    Superstructure.getInstance().removeDefaultCommand();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
