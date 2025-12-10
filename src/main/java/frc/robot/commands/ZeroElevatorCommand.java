package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ZeroElevatorCommand extends Command {
  private final boolean forced;
  private boolean shouldZero = false;

  public ZeroElevatorCommand() {
    this(false);
  }

  public ZeroElevatorCommand(boolean forced) {
    this.forced = forced;
    addRequirements(Elevator.getInstance());
  }

  @Override
  public void initialize() {
    if (Elevator.getInstance().isZeroed && !forced) {
      // If the elevator is already zeroed and we're not forcing,
      // just cancel this command
      shouldZero = false;
      cancel();
      return;
    }
    Elevator.getInstance().isZeroed = false;
    Elevator.getInstance().setZeroingVoltage();
    shouldZero = true;
  }

  @Override
  public boolean isFinished() {
    return Elevator.getInstance().getStatorCurrent() > Constants.Elevator.ZERO_MIN_CURRENT;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted && shouldZero) {
      Elevator.getInstance().stop();
      Elevator.getInstance().zero();
    }
  }
}
