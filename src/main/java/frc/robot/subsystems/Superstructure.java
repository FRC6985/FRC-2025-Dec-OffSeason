package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseScoringTracker;
import frc.robot.Robot;
import frc.robot.commands.ZeroArmCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.commands.ZeroIntakeCommand;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

public class Superstructure extends SubsystemBase {
    private static final Superstructure INSTANCE = new Superstructure();

    public static Superstructure getInstance() {
        return INSTANCE;
    }

    // Enums
    public enum State {
        StartPosition(Elevator.State.Down, Arm.PivotState.Up, Arm.RollerState.SlowIdle,
                Intake.PivotState.Up, Intake.RollerState.Off),
        Rest(Elevator.State.PreHandoff, Arm.PivotState.Down, Arm.RollerState.Idle),
        PrePopsiclePickup(Elevator.State.PreHandoff, Arm.PivotState.PopsiclePickup, Arm.RollerState.In,
                Intake.PivotState.Down, Intake.RollerState.Off),
        PopsiclePickup(Elevator.State.PopsiclePickup, Arm.PivotState.PopsiclePickup, Arm.RollerState.In,
                Intake.PivotState.Down, Intake.RollerState.Off),
        ArmSourceIntake(Elevator.State.Down, Arm.PivotState.Up, Arm.RollerState.In),
        SourceIntake(Elevator.State.SourceIntake, Arm.PivotState.Down, Arm.RollerState.Idle,
                Intake.PivotState.Up, Intake.RollerState.In),
        PreHandoff(Elevator.State.Handoff, Arm.PivotState.Down, Arm.RollerState.In,
                Intake.PivotState.Up, Intake.RollerState.In),
        Handoff(Elevator.State.Handoff, Arm.PivotState.Down, Arm.RollerState.In,
                Intake.PivotState.Up, Intake.RollerState.Out),
        PreScore(Elevator.State.PreScore, Arm.PivotState.Up, Arm.RollerState.Idle,
                Intake.PivotState.Up, Intake.RollerState.Off),
        ReverseHandoff(Elevator.State.PreHandoff, Arm.PivotState.Down, Arm.RollerState.Out,
                Intake.PivotState.Up, Intake.RollerState.In),
        PreTrough(Elevator.State.Trough, Arm.PivotState.Down, Arm.RollerState.Idle,
                Intake.PivotState.Trough, Intake.RollerState.Off),
        Trough(Elevator.State.Trough, Arm.PivotState.Down, Arm.RollerState.Idle,
                Intake.PivotState.Trough, Intake.RollerState.TroughOut),

        // L4 Score sequence
        PrepareL4(Elevator.State.L4, Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
        StartL4(Elevator.State.L4, Arm.PivotState.L4ScoreCoral, Arm.RollerState.SlowIdle),
        PlaceL4(Elevator.State.ScoreL4, Arm.PivotState.L4FinishScoreCoral, Arm.RollerState.Off),
        AfterL4(Elevator.State.PreHandoff, Arm.PivotState.Down, Arm.RollerState.SlowOut),

        // L3 Score sequence
        PrepareL3(Elevator.State.L3, Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
        StartL3(Elevator.State.L3, Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),
        PlaceL3(Elevator.State.ScoreL3, Arm.PivotState.FinishScoreCoral, Arm.RollerState.Off),
        AfterL3(Elevator.State.PostL3, Arm.PivotState.Up, Arm.RollerState.SlowOut),

        // L2 Score sequence
        PrepareL2(Elevator.State.L2, Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),
        StartL2(Elevator.State.L2, Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),
        PlaceL2(Elevator.State.ScoreL2, Arm.PivotState.FinishScoreCoral, Arm.RollerState.SlowOut),
        AfterL2(Elevator.State.PostL2, Arm.PivotState.Up, Arm.RollerState.Out),

        // Algae knocking states
        PreGetAlgae(Elevator.State.HighAlgae, Arm.PivotState.SafeInsideRobotAngle, Arm.RollerState.In),
        GetAlgae(Elevator.State.AutoAlgae, Arm.PivotState.GetAlgae, Arm.RollerState.In),
        PostGetAlgae(Elevator.State.AutoAlgae, Arm.PivotState.PostAlgae, Arm.RollerState.AlgaeIdle),
        AlgaeRest(Elevator.State.AlgaeRest, Arm.PivotState.AlgaeUp, Arm.RollerState.AlgaeIdle),
        PreBarge(Elevator.State.Barge, Arm.PivotState.PreBarge, Arm.RollerState.AlgaeIdle),
        ScoreBarge(Elevator.State.Barge, Arm.PivotState.BargeScore, Arm.RollerState.Out),
        AlgaeDescore(Elevator.State.AutoAlgae, Arm.PivotState.DescoreAlgae, Arm.RollerState.Descore),
        AlgaeExit(Elevator.State.PreHandoff, Arm.PivotState.Down, Arm.RollerState.Out),
        PreProcessor(Elevator.State.Processor, Arm.PivotState.Processor, Arm.RollerState.AlgaeIdle),
        ScoreProcessor(Elevator.State.Processor, Arm.PivotState.Processor, Arm.RollerState.SlowOut),
        PreAlgaeGroundIntake(State.Rest.elevator, Arm.PivotState.AlgaeGroundPickup, Arm.RollerState.Off,
                Intake.PivotState.Down, Intake.RollerState.Off),
        AlgaeGroundIntake(Elevator.State.GroundAlgaeIntake, Arm.PivotState.AlgaeGroundPickup, Arm.RollerState.In,
                Intake.PivotState.Down, Intake.RollerState.Off),
        ExitAlgaeGroundIntake(Elevator.State.PreHandoff, Arm.PivotState.ExitAlgaeGroundPickup,
                Arm.RollerState.AlgaeIdle,
                Intake.PivotState.Down, Intake.RollerState.Off);

        public final Elevator.State elevator;
        public final Arm.PivotState armPivot;
        public final Arm.RollerState armRollers;
        public final Intake.PivotState intakePivot;
        public final Intake.RollerState intakeRollers;

        State(Elevator.State elevator, Arm.PivotState armPivot, Arm.RollerState armRollers) {
            this(elevator, armPivot, armRollers, Intake.PivotState.OperatorControl, Intake.RollerState.OperatorControl);
        }

        State(Elevator.State elevator, Arm.PivotState armPivot, Arm.RollerState armRollers,
                Intake.PivotState intakePivot, Intake.RollerState intakeRollers) {
            this.elevator = elevator;
            this.armPivot = armPivot;
            this.armRollers = armRollers;
            this.intakePivot = intakePivot;
            this.intakeRollers = intakeRollers;
        }
    }

    public enum ScoringLevel {
        TROUGH(0),
        L2(1),
        L3(2),
        L4(3);

        public final int index;

        ScoringLevel(int index) {
            this.index = index;
        }
    }

    // Data classes
    public static class SuperstructureInputs {
        public boolean wantExtend = false;
        public boolean wantGroundIntake = false;
        public boolean wantArmSourceIntake = false;
        public boolean wantSourceIntake = false;
        public boolean wantScore = false;
        public ScoringLevel wantedScoringLevel = ScoringLevel.L4;
        public boolean wantGetAlgae = false;
        public boolean wantDescoreAlgae = false;
        public boolean wantVerticalPickup = false;
        public boolean wantResetSuperstructure = false;
        public boolean wantScoreProcessor = false;
        public boolean wantAlgaeGroundIntake = false;
        public boolean wantPopsiclePickup = false;

        public SuperstructureInputs() {
        }

        public SuperstructureInputs(boolean wantExtend, boolean wantGroundIntake, boolean wantArmSourceIntake,
                boolean wantSourceIntake, boolean wantScore, ScoringLevel wantedScoringLevel,
                boolean wantGetAlgae, boolean wantDescoreAlgae, boolean wantVerticalPickup,
                boolean wantResetSuperstructure, boolean wantScoreProcessor,
                boolean wantAlgaeGroundIntake, boolean wantPopsiclePickup) {
            this.wantExtend = wantExtend;
            this.wantGroundIntake = wantGroundIntake;
            this.wantArmSourceIntake = wantArmSourceIntake;
            this.wantSourceIntake = wantSourceIntake;
            this.wantScore = wantScore;
            this.wantedScoringLevel = wantedScoringLevel;
            this.wantGetAlgae = wantGetAlgae;
            this.wantDescoreAlgae = wantDescoreAlgae;
            this.wantVerticalPickup = wantVerticalPickup;
            this.wantResetSuperstructure = wantResetSuperstructure;
            this.wantScoreProcessor = wantScoreProcessor;
            this.wantAlgaeGroundIntake = wantAlgaeGroundIntake;
            this.wantPopsiclePickup = wantPopsiclePickup;
        }
    }

    public static class Transition {
        public State cur;
        public State next;
        public Runnable enterFunction;
        public BooleanSupplier transitionCheck;

        public Transition(State cur, State next, BooleanSupplier transitionCheck) {
            this(cur, next, () -> {
            }, transitionCheck);
        }

        public Transition(State cur, State next, Runnable enterFunction, BooleanSupplier transitionCheck) {
            this.cur = cur;
            this.next = next;
            this.enterFunction = enterFunction;
            this.transitionCheck = transitionCheck;
        }

        public boolean canTransition() {
            return cur == getInstance().state && transitionCheck.getAsBoolean();
        }
    }

    // State variables
    public SuperstructureInputs inputs = new SuperstructureInputs();
    public State state = State.StartPosition;
    public final Timer stateTimer = new Timer();
    public static final double POPSICLE_DELAY = 0.5;

    private List<Transition> transitions;

    // Constructor
    private Superstructure() {
        initializeTransitions();
    }

    private void initializeTransitions() {
        transitions = new ArrayList<>();

        // StartPosition transitions
        transitions.add(new Transition(State.StartPosition, State.Rest,
                () -> inputs.wantGroundIntake || inputs.wantArmSourceIntake));
        transitions.add(new Transition(State.StartPosition, State.PreScore,
                () -> Robot.getInstance().isAutonomous()));

        // Arm source intaking
        transitions.add(new Transition(State.Rest, State.ArmSourceIntake,
                () -> inputs.wantArmSourceIntake));
        transitions.add(new Transition(State.ArmSourceIntake, State.Rest,
                () -> !inputs.wantArmSourceIntake || Arm.getInstance().isArmStuck));

        // Intake source intaking
        transitions.add(new Transition(State.Rest, State.SourceIntake,
                () -> inputs.wantSourceIntake));
        transitions.add(new Transition(State.SourceIntake, State.Rest,
                () -> !inputs.wantSourceIntake || Intake.getInstance().hasCoral()));

        // Trough reverse handoff
        transitions.add(new Transition(State.PreScore, State.Rest,
                () -> inputs.wantedScoringLevel == ScoringLevel.TROUGH || !Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.Rest, State.ReverseHandoff,
                () -> Arm.getInstance().isAtSetpoint() && Elevator.getInstance().isAtSetpoint() &&
                        inputs.wantedScoringLevel == ScoringLevel.TROUGH && Arm.getInstance().isArmStuck &&
                        !Intake.getInstance().hasCoral() &&
                        Intake.getInstance().getEffectivePivotState() == Intake.PivotState.Up &&
                        Intake.getInstance().isAtSetpoint()));
        transitions.add(new Transition(State.ReverseHandoff, State.Rest,
                () -> Intake.getInstance().hasCoral() || inputs.wantResetSuperstructure));

        // Trough transitions
        transitions.add(new Transition(State.Rest, State.PreTrough,
                () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.TROUGH &&
                        Elevator.getInstance().isAtSetpoint() && Arm.getInstance().isAtSetpoint()));
        transitions.add(new Transition(State.PreTrough, State.Trough,
                () -> Intake.getInstance().isAtSetpoint() && inputs.wantScore));
        transitions.add(new Transition(State.PreTrough, State.Rest,
                () -> !inputs.wantExtend));
        transitions.add(new Transition(State.Trough, State.Rest,
                () -> !inputs.wantScore));

        // Handoff transitions
        transitions.add(new Transition(State.Rest, State.PreHandoff,
                () -> Elevator.getInstance().isAtSetpoint() && Arm.getInstance().isAtSetpoint() &&
                        inputs.wantedScoringLevel != ScoringLevel.TROUGH && Intake.getInstance().hasCoral()));
        transitions.add(new Transition(State.PreHandoff, State.Handoff,
                () -> Elevator.getInstance().isAtSetpoint() && Arm.getInstance().isAtSetpoint() &&
                        Intake.getInstance().isAtSetpoint()));
        transitions.add(new Transition(State.Handoff, State.Rest,
                () -> Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.Rest, State.PreScore,
                () -> Arm.getInstance().isArmStuck && inputs.wantedScoringLevel != ScoringLevel.TROUGH));
        transitions.add(new Transition(State.Handoff, State.Rest,
                () -> inputs.wantResetSuperstructure));

        // Scoring transitions
        transitions.addAll(Arrays.asList(
                scoringTransitions(ScoringLevel.L4, State.PrepareL4, State.StartL4, State.PlaceL4, State.AfterL4)));
        transitions.addAll(Arrays.asList(
                scoringTransitions(ScoringLevel.L3, State.PrepareL3, State.StartL3, State.PlaceL3, State.AfterL3)));
        transitions.addAll(Arrays.asList(
                scoringTransitions(ScoringLevel.L2, State.PrepareL2, State.StartL2, State.PlaceL2, State.AfterL2)));

        // Algae removal
        transitions.add(new Transition(State.AlgaeExit, State.PreGetAlgae,
                () -> inputs.wantGetAlgae && !Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.Rest, State.PreGetAlgae,
                () -> inputs.wantGetAlgae && !Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.PreGetAlgae, State.Rest,
                () -> !inputs.wantGetAlgae));
        transitions.add(new Transition(State.PreGetAlgae, State.GetAlgae,
                () -> Elevator.getInstance().isAtSetpoint()));
        transitions.add(new Transition(State.GetAlgae, State.PreGetAlgae,
                () -> !inputs.wantGetAlgae));
        transitions.add(new Transition(State.GetAlgae, State.PostGetAlgae,
                () -> Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.PostGetAlgae, State.AlgaeRest,
                () -> Arm.getInstance().isAtSetpoint() && Arm.getInstance().atSafeReefDistance()));
        transitions.add(new Transition(State.AlgaeRest, State.AlgaeExit,
                () -> !Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.AlgaeExit, State.Rest,
                () -> Arm.getInstance().isAtSetpoint() && Elevator.getInstance().isAtSetpoint()));

        // Barge transitions
        transitions.add(new Transition(State.AlgaeRest, State.PreBarge,
                () -> inputs.wantExtend));
        transitions.add(new Transition(State.PreBarge, State.AlgaeRest,
                () -> !inputs.wantExtend));
        transitions.add(new Transition(State.PreBarge, State.ScoreBarge,
                () -> inputs.wantScore && PoseScoringTracker.atGoodScoringDistance()));
        transitions.add(new Transition(State.ScoreBarge, State.PreBarge,
                () -> (!inputs.wantExtend || !Arm.getInstance().isArmStuck)
                        && Arm.getInstance().atSafeBargeDistance()));

        // Algae descore
        transitions.add(new Transition(State.Rest, State.AlgaeDescore,
                () -> inputs.wantDescoreAlgae));
        transitions.add(new Transition(State.AlgaeDescore, State.Rest,
                () -> !inputs.wantDescoreAlgae));

        // Processor transitions
        transitions.add(new Transition(State.AlgaeRest, State.PreProcessor,
                () -> inputs.wantScoreProcessor));
        transitions.add(new Transition(State.PreProcessor, State.ScoreProcessor,
                () -> inputs.wantScore));
        transitions.add(new Transition(State.ScoreProcessor, State.AlgaeRest,
                () -> !inputs.wantScoreProcessor && !Arm.getInstance().isArmStuck &&
                        Arm.getInstance().atSafeProcessorDistance()));
        transitions.add(new Transition(State.PreProcessor, State.AlgaeRest,
                () -> !inputs.wantScoreProcessor && Arm.getInstance().atSafeProcessorDistance()));

        // Algae ground intake
        transitions.add(new Transition(State.Rest, State.PreAlgaeGroundIntake,
                () -> inputs.wantAlgaeGroundIntake && !Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.PreAlgaeGroundIntake, State.AlgaeGroundIntake,
                () -> inputs.wantAlgaeGroundIntake && Intake.getInstance().isAtSetpoint()));
        transitions.add(new Transition(State.AlgaeGroundIntake, State.ExitAlgaeGroundIntake,
                () -> !inputs.wantAlgaeGroundIntake || Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.ExitAlgaeGroundIntake, State.AlgaeRest,
                () -> Elevator.getInstance().isAtSetpoint() && Arm.getInstance().isAtSetpoint()));

        // Popsicle pickup
        transitions.add(new Transition(State.PopsiclePickup, State.PrePopsiclePickup,
                () -> !inputs.wantPopsiclePickup
                        || (Robot.getInstance().isAutonomous() && stateTimer.hasElapsed(POPSICLE_DELAY))));
        transitions.add(new Transition(State.PrePopsiclePickup, State.Rest,
                () -> !inputs.wantPopsiclePickup));
        transitions.add(new Transition(State.Rest, State.PrePopsiclePickup,
                () -> inputs.wantPopsiclePickup && !Arm.getInstance().isArmStuck));
        transitions.add(new Transition(State.PrePopsiclePickup, State.PopsiclePickup,
                () -> inputs.wantPopsiclePickup && Intake.getInstance().isAtSetpoint()));
        transitions.add(new Transition(State.PopsiclePickup, State.PreScore,
                () -> stateTimer.hasElapsed(POPSICLE_DELAY) && Arm.getInstance().isArmStuck));
    }

    private Transition[] scoringTransitions(ScoringLevel level, State prepare, State start,
            State place, State after) {
        return new Transition[] {
                new Transition(prepare, State.PreScore,
                        () -> Arm.getInstance().isAtSetpoint() &&
                                (!Arm.getInstance().isArmStuck || !inputs.wantExtend ||
                                        inputs.wantedScoringLevel != level)),
                new Transition(start, prepare,
                        () -> !inputs.wantExtend || inputs.wantedScoringLevel != level ||
                                !Arm.getInstance().isArmStuck),
                new Transition(prepare, start,
                        () -> Elevator.getInstance().isLazierAtSetpoint() &&
                                Arm.getInstance().isArmStuck && inputs.wantExtend &&
                                inputs.wantedScoringLevel == level),
                // ← BURAYA enterFunction ekle: markPoseScored()
                new Transition(start, place,
                        () -> PoseScoringTracker.markPoseScored(), // ← Enter function
                        () -> Elevator.getInstance().isAtSetpoint() &&
                                Arm.getInstance().isAtSetpoint() &&
                                inputs.wantScore),
                new Transition(place, after,
                        () -> Elevator.getInstance().isAtSetpoint() &&
                                Arm.getInstance().isAtSetpoint() &&
                                (place == State.PlaceL2 || place == State.PlaceL3
                                        ? Arm.getInstance().atSafePlacementDistance()
                                        : true)),
                new Transition(after, State.Rest,
                        () -> Arm.getInstance().isInsideFrame())
        };
    }

    public ParallelCommandGroup makeZeroAllSubsystemsCommand() {
        return new ParallelCommandGroup(
                new ZeroIntakeCommand(),
                new ZeroElevatorCommand(),
                new ZeroArmCommand());
    }

    public void emptyInputs() {
        inputs = new SuperstructureInputs();
    }

    public void setStates() {
        Arm.getInstance().setState(state.armPivot, state.armRollers);
        Elevator.getInstance().state = state.elevator;
        Intake.getInstance().setState(state.intakePivot, state.intakeRollers);
    }

    @Override
    public void periodic() {
        if (!Elevator.getInstance().isZeroed || !Intake.getInstance().isZeroed ||
                !Arm.getInstance().isZeroed) {
            return;
        }

        stateTimer.start();

        for (Transition transition : transitions) {
            if (transition.canTransition()) {
                setState(transition.next);
                transition.enterFunction.run();
                setStates();
                return;
            }
        }
    }

    public void setState(State newState) {
        if (newState != state) {
            stateTimer.restart();
        }
        state = newState;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", () -> state.toString(), null);
        builder.addBooleanProperty("wantExtend", () -> inputs.wantExtend, null);
        builder.addBooleanProperty("wantScore", () -> inputs.wantScore, null);
        builder.addStringProperty("scoringLevel", () -> inputs.wantedScoringLevel.toString(), null);
    }
}