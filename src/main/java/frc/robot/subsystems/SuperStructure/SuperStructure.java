package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;

public class SuperStructure extends SubsystemBase {

    private final SuperstructureTransitionTable transitionTable;

    private SuperstructureState state = SuperstructureState.StartPosition;
    public final Timer timer = new Timer();

    private SuperstructureInputs inputs = new SuperstructureInputs();
    private final Arm arm;
    private final Elevator elevator;
    private final Intake intake;

    public SuperStructure(RobotContainer rC) {
        this.transitionTable = new SuperstructureTransitionTable(this, rC);
        arm = rC.subsystems.arm;
        elevator = rC.subsystems.elevator;
        intake = rC.subsystems.intake;
        timer.start();
    }

    @Override
    public void periodic() {

        for (SuperstructureTransition t : transitionTable.getTransitions()) {
            if (t.current == state && t.condition.check()) {
                state = t.next;
                t.enterFunction.run();
                applyState();
                break;
            }
        }
    }

    public void applyState() {
        arm.setState(state.armPivot, state.armRollers);
        elevator.setState(state.elevator);
        intake.setState(state.intakePivot, state.intakeRollers);
    }

    public SuperstructureInputs getInputs() {
        return inputs;
    }
}
