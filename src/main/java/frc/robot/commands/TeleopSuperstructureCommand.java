package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.Superstructure;

public class TeleopSuperstructureCommand extends Command {

  public TeleopSuperstructureCommand() {
    addRequirements(Superstructure.getInstance());
  }

  @Override
  public void execute() {
    Superstructure.getInstance().inputs = Controls.getInstance().getSuperstructureInputs();
  }

  @Override
  public void end(boolean interrupted) {
    Superstructure.getInstance().emptyInputs();
  }
}