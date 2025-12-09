package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ZeroIntakeCommand extends Command {
    private final boolean forced;
    private boolean shouldZero = false;

    public ZeroIntakeCommand() {
        this(false);
    }

    public ZeroIntakeCommand(boolean forced) {
        this.forced = forced;
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        if (Intake.getInstance().isZeroed && !forced) {
            // If the intake is already zeroed and we're not forcing,
            // just cancel this command
            shouldZero = false;
            cancel();
            return;
        }
        Intake.getInstance().isZeroed = false;
        Intake.getInstance().setZeroingVoltage();
        shouldZero = true;
    }

    @Override
    public boolean isFinished() {
        return Intake.getInstance().getVelocity() < Constants.Intake.ZERO_MIN_CURRENT;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted && shouldZero) {
            Intake.getInstance().stop();
            Intake.getInstance().zero();
        }
    }
}