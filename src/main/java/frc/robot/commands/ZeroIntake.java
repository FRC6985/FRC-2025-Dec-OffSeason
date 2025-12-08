package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Intake;

public class ZeroIntake extends Command {
  public final Intake intake;
  private boolean shouldZero = false;

  public ZeroIntake(Intake _intake) {
    intake = _intake;
    addRequirements(_intake);
  }

  @Override
  public void initialize() {
    super.initialize();
    if (intake.isZeroed()) {
      shouldZero = false;
      this.cancel();
      return;
    }
    intake.setZeroed(false);
    intake.setZeroingVoltage();
    shouldZero = true;
  }

  @Override
  public boolean isFinished() {
    return intake.getVelocity() < Constants.Intake.ZERO_MIN_CURRENT;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted && shouldZero) {
      intake.stop();
      intake.zero();
    }
  }
}
