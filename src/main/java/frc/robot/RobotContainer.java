package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.SubSystems;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
  public final SubSystems subsystems;
  public final CommandXboxController driver = new CommandXboxController(0);

  public boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
  }

  public boolean isOnRedSide() {
    return subsystems.drive.poseEstimator.getEstimatedPosition().getX() > (Constants.Field.FIELD_X_SIZE / 2);
  }

  public RobotContainer(SubSystems subsystems) {
    this.subsystems = subsystems;
    confirgureButtonBindings(subsystems.drive);
  }

  public void confirgureButtonBindings(Drive drive) {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
  }

  public void periodic() {
  }
}
