package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;

public class ZeroArm extends Command {
  public final Arm arm;

  public ZeroArm(Arm _arm) {
    arm = _arm;
    addRequirements(_arm);
  }

  @Override
  public void initialize() {
    super.initialize();
    if (arm.isZeroed()) {
      this.cancel();
      return;
    }
    arm.resetRelativeFromAbsolute();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
