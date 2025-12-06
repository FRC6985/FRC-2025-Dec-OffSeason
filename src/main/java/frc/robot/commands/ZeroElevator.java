package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.Elevator;

public class ZeroElevator extends Command {
  public final Elevator elevator;
  private boolean shouldZero = false;

  public ZeroElevator(Elevator _elevator) {
    elevator = _elevator;
    addRequirements(_elevator);
  }

  @Override
  public void initialize() {
    super.initialize();
    if (elevator.isZeroed()) {
      shouldZero = false;
      this.cancel();
      return;
    }
    elevator.setZeroed(false);
    elevator.setZeroingVoltage();
    shouldZero = true;
  }

  @Override
  public boolean isFinished() {
    return elevator.getStatorCurrent() < Constants.Elevator.ZERO_MIN_CURRENT;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted && shouldZero) {
      elevator.stop();
      elevator.zero();
    }
  }
}
