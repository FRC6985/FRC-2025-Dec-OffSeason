package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Intake.PivotState;
import frc.robot.Constants.Intake.RollerState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.SubSystems;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
  public final SubSystems subsystems;
  public final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer(SubSystems subsystems) {
    this.subsystems = subsystems;
    confirgureButtonBindings(subsystems.drive);
  }

  public void confirgureButtonBindings(Drive drive) {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    driver.x().onTrue(new InstantCommand(() -> {
      subsystems.intake.setState(PivotState.Down, RollerState.In);
    }));
    driver.y().onTrue(new InstantCommand(() -> {
      subsystems.intake.setState(PivotState.Up, RollerState.Out);
    }));
  }
}
