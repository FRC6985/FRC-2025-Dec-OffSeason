package frc.robot.subsystems.SuperStructure;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.SuperStructure.SuperstructureInputs.ScoringLevel;
import frc.robot.Constants;

public class SuperstructureTransitionTable {
    private final SuperStructure ss;
    private final RobotContainer rC;
    private final List<SuperstructureTransition> transitions = new ArrayList<>();
    private final Arm Arm;
    private final Elevator Elevator;
    private final Intake Intake;

    public SuperstructureTransitionTable(SuperStructure ss, RobotContainer rC) {
        this.Arm = rC.subsystems.arm;
        this.Elevator = rC.subsystems.elevator;
        this.Intake = rC.subsystems.intake;
        this.ss = ss;
        this.rC = rC;
        // ----------------------------------------------------
        // NORMAL TRANSITIONS
        // ----------------------------------------------------

        add(SuperstructureState.StartPosition, SuperstructureState.Rest,
                () -> ss.getInputs().wantGroundIntake || ss.getInputs().wantArmSourceIntake);

        add(SuperstructureState.StartPosition, SuperstructureState.PreScore,
                () -> rC.robot.isAutonomous());

        // Arm source intake
        add(SuperstructureState.Rest, SuperstructureState.ArmSourceIntake,
                () -> ss.getInputs().wantArmSourceIntake);

        add(SuperstructureState.ArmSourceIntake, SuperstructureState.Rest,
                () -> !ss.getInputs().wantArmSourceIntake || Arm.getHasObject());

        // Intake source intake
        add(SuperstructureState.Rest, SuperstructureState.SourceIntake,
                () -> ss.getInputs().wantSourceIntake);

        add(SuperstructureState.SourceIntake, SuperstructureState.Rest,
                () -> !ss.getInputs().wantSourceIntake || Intake.hasCoral());

        // Trough reverse handoff
        add(SuperstructureState.PreScore, SuperstructureState.Rest,
                () -> ss.getInputs().wantedScoringLevel == ScoringLevel.TROUGH || !Arm.getHasObject());

        add(SuperstructureState.Rest, SuperstructureState.ReverseHandoff,
                () -> Arm.getAtSetpoint() && Elevator.atSetpoint() &&
                        ss.getInputs().wantedScoringLevel == ScoringLevel.TROUGH &&
                        Arm.getHasObject() && !Intake.hasCoral() &&
                        Intake.getEffectivePivotState() == Constants.Intake.PivotState.Up &&
                        Intake.atSetpoint());

        add(SuperstructureState.ReverseHandoff, SuperstructureState.Rest,
                () -> Intake.hasCoral() || ss.getInputs().wantResetSuperstructure);

        // Trough scoring
        add(SuperstructureState.Rest, SuperstructureState.PreTrough,
                () -> ss.getInputs().wantExtend &&
                        ss.getInputs().wantedScoringLevel == ScoringLevel.TROUGH &&
                        Elevator.atSetpoint && Arm.atSetpoint);

        add(SuperstructureState.PreTrough, SuperstructureState.Trough,
                () -> Intake.atSetpoint && ss.getInputs().wantScore);

        add(SuperstructureState.PreTrough, SuperstructureState.Rest,
                () -> !ss.getInputs().wantExtend);

        add(SuperstructureState.Trough, SuperstructureState.Rest,
                () -> !ss.getInputs().wantScore);

        // PreHandoff
        add(SuperstructureState.Rest, SuperstructureState.PreHandoff,
                () -> Elevator.atSetpoint && Arm.atSetpoint &&
                        ss.getInputs().wantedScoringLevel != ScoringLevel.TROUGH &&
                        Intake.hasCoral);

        // Handoff
        add(SuperstructureState.PreHandoff, SuperstructureState.Handoff,
                () -> Elevator.atSetpoint && Arm.atSetpoint && Intake.atSetpoint);

        add(SuperstructureState.Handoff, SuperstructureState.Rest,
                () -> Arm.hasObject);

        add(SuperstructureState.Rest, SuperstructureState.PreScore,
                () -> Arm.hasObject &&
                        ss.getInputs().wantedScoringLevel != ScoringLevel.TROUGH);

        add(SuperstructureState.Handoff, SuperstructureState.Rest,
                () -> ss.getInputs().wantResetSuperstructure);

        // Scoring prepare transitions
        add(SuperstructureState.PreScore, SuperstructureState.PrepareL4,
                () -> ss.getInputs().wantExtend &&
                        ss.getInputs().wantedScoringLevel == ScoringLevel.L4);

        add(SuperstructureState.PreScore, SuperstructureState.PrepareL3,
                () -> ss.getInputs().wantExtend &&
                        ss.getInputs().wantedScoringLevel == ScoringLevel.L3);

        add(SuperstructureState.PreScore, SuperstructureState.PrepareL2,
                () -> ss.getInputs().wantExtend &&
                        ss.getInputs().wantedScoringLevel == ScoringLevel.L2);

        // ----------------------------------------------------
        // SCORING STATE GROUPS
        // ----------------------------------------------------

        addScoringTransitions(ScoringLevel.L4,
                SuperstructureState.PrepareL4, SuperstructureState.StartL4, SuperstructureState.PlaceL4,
                SuperstructureState.AfterL4);

        addScoringTransitions(ScoringLevel.L3,
                SuperstructureState.PrepareL3, SuperstructureState.StartL3, SuperstructureState.PlaceL3,
                SuperstructureState.AfterL3);

        addScoringTransitions(ScoringLevel.L2,
                SuperstructureState.PrepareL2, SuperstructureState.StartL2, SuperstructureState.PlaceL2,
                SuperstructureState.AfterL2);

        // ----------------------------------------------------
        // ALGAE FLOW
        // ----------------------------------------------------

        add(SuperstructureState.AlgaeExit, SuperstructureState.PreGetAlgae,
                () -> ss.getInputs().wantGetAlgae && !Arm.hasObject);

        add(SuperstructureState.Rest, SuperstructureState.PreGetAlgae,
                () -> ss.getInputs().wantGetAlgae && !Arm.hasObject);

        add(SuperstructureState.PreGetAlgae, SuperstructureState.Rest,
                () -> !ss.getInputs().wantGetAlgae);

        add(SuperstructureState.PreGetAlgae, SuperstructureState.GetAlgae,
                () -> Elevator.atSetpoint);

        add(SuperstructureState.GetAlgae, SuperstructureState.PreGetAlgae,
                () -> !ss.getInputs().wantGetAlgae);

        add(SuperstructureState.GetAlgae, SuperstructureState.PostGetAlgae,
                () -> Arm.hasObject);

        add(SuperstructureState.PostGetAlgae, SuperstructureState.AlgaeRest,
                () -> Arm.atSetpoint && Arm.atSafeReefDistance());

        add(SuperstructureState.AlgaeRest, SuperstructureState.AlgaeExit,
                () -> !Arm.hasObject);

        add(SuperstructureState.AlgaeExit, SuperstructureState.Rest,
                () -> Arm.atSetpoint && Elevator.atSetpoint);

        // Algae → Barge
        add(SuperstructureState.AlgaeRest, SuperstructureState.PreBarge,
                () -> ss.getInputs().wantExtend);

        add(SuperstructureState.PreBarge, SuperstructureState.AlgaeRest,
                () -> !ss.getInputs().wantExtend);

        add(SuperstructureState.PreBarge, SuperstructureState.ScoreBarge,
                () -> ss.getInputs().wantScore && Swerve.atGoodScoringDistance);

        add(SuperstructureState.ScoreBarge, SuperstructureState.PreBarge,
                () -> (!ss.getInputs().wantExtend || !Arm.hasObject) &&
                        Arm.atSafeBargeDistance());

        // Algae Descore
        add(SuperstructureState.Rest, SuperstructureState.AlgaeDescore,
                () -> ss.getInputs().wantDescoreAlgae);

        add(SuperstructureState.AlgaeDescore, SuperstructureState.Rest,
                () -> !ss.getInputs().wantDescoreAlgae);

        // Processor scoring
        add(SuperstructureState.AlgaeRest, SuperstructureState.PreProcessor,
                () -> ss.getInputs().wantScoreProcessor);

        add(SuperstructureState.PreProcessor, SuperstructureState.ScoreProcessor,
                () -> ss.getInputs().wantScore);

        add(SuperstructureState.ScoreProcessor, SuperstructureState.AlgaeRest,
                () -> !ss.getInputs().wantScoreProcessor &&
                        !Arm.hasObject &&
                        Arm.atSafeProcessorDistance());

        add(SuperstructureState.PreProcessor, SuperstructureState.AlgaeRest,
                () -> !ss.getInputs().wantScoreProcessor &&
                        Arm.atSafeProcessorDistance());

        // Ground algae intake
        add(SuperstructureState.Rest, SuperstructureState.PreAlgaeGroundIntake,
                () -> ss.getInputs().wantAlgaeGroundIntake && !Arm.hasObject);

        add(SuperstructureState.PreAlgaeGroundIntake, SuperstructureState.AlgaeGroundIntake,
                () -> ss.getInputs().wantAlgaeGroundIntake && Intake.atSetpoint);

        add(SuperstructureState.AlgaeGroundIntake, SuperstructureState.ExitAlgaeGroundIntake,
                () -> !ss.getInputs().wantAlgaeGroundIntake || Arm.hasObject);

        add(SuperstructureState.ExitAlgaeGroundIntake, SuperstructureState.AlgaeRest,
                () -> Elevator.atSetpoint && Arm.atSetpoint);

        // Popsicle pickup
        add(SuperstructureState.PopsiclePickup, SuperstructureState.PrePopsiclePickup,
                () -> !ss.getInputs().wantPopsiclePickup ||
                        (Robot.isAutonomous &&
                                ss.stateTimer.hasElapsed(POPSICLE_DELAY)));

        add(SuperstructureState.PrePopsiclePickup, SuperstructureState.Rest,
                () -> !ss.getInputs().wantPopsiclePickup);

        add(SuperstructureState.Rest, SuperstructureState.PrePopsiclePickup,
                () -> ss.getInputs().wantPopsiclePickup && !Arm.hasObject);

        add(SuperstructureState.PrePopsiclePickup, SuperstructureState.PopsiclePickup,
                () -> ss.getInputs().wantPopsiclePickup && Intake.atSetpoint);

        add(SuperstructureState.PopsiclePickup, SuperstructureState.PreScore,
                () -> ss.stateTimer.hasElapsed(POPSICLE_DELAY) && Arm.hasObject);
    }

    // ----------------------------------------------------
    // Helper: scoringTransitions() (Kotlin bire bir)
    // ----------------------------------------------------

    private void addScoringTransitions(
            ScoringLevel lvl,
            SuperstructureState prepare,
            SuperstructureState start,
            SuperstructureState place,
            SuperstructureState after) {
        // Exit paths
        add(prepare, SuperstructureState.PreScore,
                () -> Arm.atSetpoint &&
                        (!Arm.hasObject ||
                                !ss.getInputs().wantExtend ||
                                ss.getInputs().wantedScoringLevel != lvl));

        add(start, prepare,
                () -> !ss.getInputs().wantExtend ||
                        ss.getInputs().wantedScoringLevel != lvl ||
                        !Arm.hasObject);

        // Main transitions
        add(prepare, start,
                () -> Elevator.lazierAtSetpoint &&
                        Arm.hasObject &&
                        ss.getInputs().wantExtend &&
                        ss.getInputs().wantedScoringLevel == lvl);

        add(start, place,
                () -> Elevator.atSetpoint && Arm.atSetpoint && ss.getInputs().wantScore,
                () -> Swerve.markPoseScored());

        add(place, after,
                () -> Elevator.atSetpoint &&
                        Arm.atSetpoint &&
                        ((place == SuperstructureState.PlaceL2 || place == SuperstructureState.PlaceL3)
                                ? Arm.atSafePlacementDistance()
                                : true));

        add(after, SuperstructureState.Rest,
                () -> Arm.insideFrame);
    }

    // ----------------------------------------------------
    // Simpler add() overloads
    // ----------------------------------------------------

    private void add(SuperstructureState cur, SuperstructureState next, Condition cond) {
        transitions.add(new SuperstructureTransition(cur, next, cond, () -> {
        }));
    }

    private void add(SuperstructureState cur, SuperstructureState next, Condition cond, Runnable enter) {
        transitions.add(new SuperstructureTransition(cur, next, cond, enter));
    }

    public List<SuperstructureTransition> getTransitions() {
        return transitions;
    }

    // Functional interface
    public interface Condition {
        boolean check();
    }

    // Short alias
    private static SuperstructureState SuperstructureState = null;
}
