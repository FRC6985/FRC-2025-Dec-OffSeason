package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.LoggedRobot;

public class SuperstructureO extends SubsystemBase {

        // Subsystem references
        private final Arm arm;
        private final Elevator elevator;
        private final Intake intake;
        private final LoggedRobot robot;

        public enum ScoringLevel {
                TROUGH(0),
                L2(1),
                L3(2),
                L4(3);

                private final int index;

                ScoringLevel(int index) {
                        this.index = index;
                }

                public int getIndex() {
                        return index;
                }
        }

        // Robot State Enum
        public enum State {
                StartPosition(Constants.Elevator.State.Down, Constants.Arm.PivotState.Up,
                                Constants.Arm.RollerState.SlowIdle, Constants.Intake.PivotState.Up,
                                Constants.Intake.RollerState.Off),
                Rest(Constants.Elevator.State.PreHandoff, Constants.Arm.PivotState.Down, Constants.Arm.RollerState.Idle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PrePopsiclePickup(Constants.Elevator.State.PreHandoff, Constants.Arm.PivotState.PopsiclePickup,
                                Constants.Arm.RollerState.In,
                                Constants.Intake.PivotState.Down, Constants.Intake.RollerState.Off),
                PopsiclePickup(Constants.Elevator.State.PopsiclePickup, Constants.Arm.PivotState.PopsiclePickup,
                                Constants.Arm.RollerState.In,
                                Constants.Intake.PivotState.Down, Constants.Intake.RollerState.Off),
                ArmSourceIntake(Constants.Elevator.State.Down, Constants.Arm.PivotState.Up,
                                Constants.Arm.RollerState.In,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                SourceIntake(Constants.Elevator.State.SourceIntake, Constants.Arm.PivotState.Down,
                                Constants.Arm.RollerState.Idle,
                                Constants.Intake.PivotState.Up, Constants.Intake.RollerState.In),
                PreHandoff(Constants.Elevator.State.Handoff, Constants.Arm.PivotState.Down,
                                Constants.Arm.RollerState.In, Constants.Intake.PivotState.Up,
                                Constants.Intake.RollerState.In),
                Handoff(Constants.Elevator.State.Handoff, Constants.Arm.PivotState.Down, Constants.Arm.RollerState.In,
                                Constants.Intake.PivotState.Up,
                                Constants.Intake.RollerState.Out),
                PreScore(Constants.Elevator.State.PreScore, Constants.Arm.PivotState.Up, Constants.Arm.RollerState.Idle,
                                Constants.Intake.PivotState.Up,
                                Constants.Intake.RollerState.Off),
                ReverseHandoff(Constants.Elevator.State.PreHandoff, Constants.Arm.PivotState.Down,
                                Constants.Arm.RollerState.Out,
                                Constants.Intake.PivotState.Up, Constants.Intake.RollerState.In),
                PreTrough(Constants.Elevator.State.Trough, Constants.Arm.PivotState.Down,
                                Constants.Arm.RollerState.Idle, Constants.Intake.PivotState.Trough,
                                Constants.Intake.RollerState.Off),
                Trough(Constants.Elevator.State.Trough, Constants.Arm.PivotState.Down, Constants.Arm.RollerState.Idle,
                                Constants.Intake.PivotState.Trough,
                                Constants.Intake.RollerState.TroughOut),

                // L4 Scoring
                PrepareL4(Constants.Elevator.State.L4, Constants.Arm.PivotState.AboveScoreCoral,
                                Constants.Arm.RollerState.Idle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                StartL4(Constants.Elevator.State.L4, Constants.Arm.PivotState.L4ScoreCoral,
                                Constants.Arm.RollerState.SlowIdle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PlaceL4(Constants.Elevator.State.ScoreL4, Constants.Arm.PivotState.L4FinishScoreCoral,
                                Constants.Arm.RollerState.Off,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                AfterL4(Constants.Elevator.State.PreHandoff, Constants.Arm.PivotState.Down,
                                Constants.Arm.RollerState.SlowOut,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),

                // L3 Scoring
                PrepareL3(Constants.Elevator.State.L3, Constants.Arm.PivotState.AboveScoreCoral,
                                Constants.Arm.RollerState.Idle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                StartL3(Constants.Elevator.State.L3, Constants.Arm.PivotState.ScoreCoral,
                                Constants.Arm.RollerState.SlowIdle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PlaceL3(Constants.Elevator.State.ScoreL3, Constants.Arm.PivotState.FinishScoreCoral,
                                Constants.Arm.RollerState.Off,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                AfterL3(Constants.Elevator.State.PostL3, Constants.Arm.PivotState.Up, Constants.Arm.RollerState.SlowOut,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),

                // L2 Scoring
                PrepareL2(Constants.Elevator.State.L2, Constants.Arm.PivotState.AboveScoreCoral,
                                Constants.Arm.RollerState.Idle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                StartL2(Constants.Elevator.State.L2, Constants.Arm.PivotState.ScoreCoral,
                                Constants.Arm.RollerState.SlowIdle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PlaceL2(Constants.Elevator.State.ScoreL2, Constants.Arm.PivotState.FinishScoreCoral,
                                Constants.Arm.RollerState.SlowOut,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                AfterL2(Constants.Elevator.State.PostL2, Constants.Arm.PivotState.Up, Constants.Arm.RollerState.Out,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),

                // Algae States
                PreGetAlgae(Constants.Elevator.State.HighAlgae, Constants.Arm.PivotState.SafeInsideRobotAngle,
                                Constants.Arm.RollerState.In,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                GetAlgae(Constants.Elevator.State.AutoAlgae, Constants.Arm.PivotState.GetAlgae,
                                Constants.Arm.RollerState.In,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PostGetAlgae(Constants.Elevator.State.AutoAlgae, Constants.Arm.PivotState.PostAlgae,
                                Constants.Arm.RollerState.AlgaeIdle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                AlgaeRest(Constants.Elevator.State.AlgaeRest, Constants.Arm.PivotState.AlgaeUp,
                                Constants.Arm.RollerState.AlgaeIdle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PreBarge(Constants.Elevator.State.Barge, Constants.Arm.PivotState.PreBarge,
                                Constants.Arm.RollerState.AlgaeIdle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                ScoreBarge(Constants.Elevator.State.Barge, Constants.Arm.PivotState.BargeScore,
                                Constants.Arm.RollerState.Out,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                AlgaeDescore(Constants.Elevator.State.AutoAlgae, Constants.Arm.PivotState.DescoreAlgae,
                                Constants.Arm.RollerState.Descore,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                AlgaeExit(Constants.Elevator.State.PreHandoff, Constants.Arm.PivotState.Down,
                                Constants.Arm.RollerState.Out,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PreProcessor(Constants.Elevator.State.Processor, Constants.Arm.PivotState.Processor,
                                Constants.Arm.RollerState.AlgaeIdle,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                ScoreProcessor(Constants.Elevator.State.Processor, Constants.Arm.PivotState.Processor,
                                Constants.Arm.RollerState.SlowOut,
                                Constants.Intake.PivotState.OperatorControl,
                                Constants.Intake.RollerState.OperatorControl),
                PreAlgaeGroundIntake(Constants.Elevator.State.PreHandoff, Constants.Arm.PivotState.AlgaeGroundPickup,
                                Constants.Arm.RollerState.Off,
                                Constants.Intake.PivotState.Down, Constants.Intake.RollerState.Off),
                AlgaeGroundIntake(Constants.Elevator.State.GroundAlgaeIntake,
                                Constants.Arm.PivotState.AlgaeGroundPickup,
                                Constants.Arm.RollerState.In, Constants.Intake.PivotState.Down,
                                Constants.Intake.RollerState.Off),
                ExitAlgaeGroundIntake(Constants.Elevator.State.PreHandoff,
                                Constants.Arm.PivotState.ExitAlgaeGroundPickup,
                                Constants.Arm.RollerState.AlgaeIdle, Constants.Intake.PivotState.Down,
                                Constants.Intake.RollerState.Off);

                private final Constants.Elevator.State elevator;
                private final Constants.Arm.PivotState armPivot;
                private final Constants.Arm.RollerState armRollers;
                private final Constants.Intake.PivotState intakePivot;
                private final Constants.Intake.RollerState intakeRollers;

                State(Constants.Elevator.State elevator, Constants.Arm.PivotState armPivot,
                                Constants.Arm.RollerState armRollers) {
                        this(elevator, armPivot, armRollers, Constants.Intake.PivotState.OperatorControl,
                                        Constants.Intake.RollerState.OperatorControl);
                }

                State(Constants.Elevator.State elevator, Constants.Arm.PivotState armPivot,
                                Constants.Arm.RollerState armRollers,
                                Constants.Intake.PivotState intakePivot, Constants.Intake.RollerState intakeRollers) {
                        this.elevator = elevator;
                        this.armPivot = armPivot;
                        this.armRollers = armRollers;
                        this.intakePivot = intakePivot;
                        this.intakeRollers = intakeRollers;
                }

                public Constants.Elevator.State getElevator() {
                        return elevator;
                }

                public Constants.Arm.PivotState getArmPivot() {
                        return armPivot;
                }

                public Constants.Arm.RollerState getArmRollers() {
                        return armRollers;
                }

                public Constants.Intake.PivotState getIntakePivot() {
                        return intakePivot;
                }

                public Constants.Intake.RollerState getIntakeRollers() {
                        return intakeRollers;
                }
        }

        // Superstructure Inputs Data Class
        public static class SuperstructureInputs {
                public final boolean wantExtend;
                public final boolean wantGroundIntake;
                public final boolean wantArmSourceIntake;
                public final boolean wantSourceIntake;
                public final boolean wantScore;
                public final ScoringLevel wantedScoringLevel;
                public final boolean wantGetAlgae;
                public final boolean wantDescoreAlgae;
                public final boolean wantVerticalPickup;
                public final boolean wantResetSuperstructure;
                public final boolean wantScoreProcessor;
                public final boolean wantAlgaeGroundIntake;
                public final boolean wantPopsiclePickup;

                public SuperstructureInputs() {
                        this(false, false, false, false, false, ScoringLevel.L4, false, false, false, false, false,
                                        false, false);
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

                public SuperstructureInputs copy(boolean wantExtend, boolean wantGroundIntake,
                                boolean wantArmSourceIntake,
                                boolean wantSourceIntake, boolean wantScore, ScoringLevel wantedScoringLevel,
                                boolean wantGetAlgae, boolean wantDescoreAlgae, boolean wantVerticalPickup,
                                boolean wantResetSuperstructure, boolean wantScoreProcessor,
                                boolean wantAlgaeGroundIntake, boolean wantPopsiclePickup) {
                        return new SuperstructureInputs(wantExtend, wantGroundIntake, wantArmSourceIntake,
                                        wantSourceIntake,
                                        wantScore, wantedScoringLevel, wantGetAlgae, wantDescoreAlgae,
                                        wantVerticalPickup,
                                        wantResetSuperstructure, wantScoreProcessor, wantAlgaeGroundIntake,
                                        wantPopsiclePickup);
                }

                @Override
                public String toString() {
                        return "SuperstructureInputs(" +
                                        "wantExtend=" + wantExtend +
                                        ", wantGroundIntake=" + wantGroundIntake +
                                        ", wantArmSourceIntake=" + wantArmSourceIntake +
                                        ", wantSourceIntake=" + wantSourceIntake +
                                        ", wantScore=" + wantScore +
                                        ", wantedScoringLevel=" + wantedScoringLevel +
                                        ", wantGetAlgae=" + wantGetAlgae +
                                        ", wantDescoreAlgae=" + wantDescoreAlgae +
                                        ", wantVerticalPickup=" + wantVerticalPickup +
                                        ", wantResetSuperstructure=" + wantResetSuperstructure +
                                        ", wantScoreProcessor=" + wantScoreProcessor +
                                        ", wantAlgaeGroundIntake=" + wantAlgaeGroundIntake +
                                        ", wantPopsiclePickup=" + wantPopsiclePickup +
                                        ')';
                }
        }

        // Transition Class
        public static class Transition {
                private final State cur;
                private final State next;
                private final Runnable enterFunction;
                private final BooleanSupplier transitionCheck;

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

                public State getCur() {
                        return cur;
                }

                public State getNext() {
                        return next;
                }

                public Runnable getEnterFunction() {
                        return enterFunction;
                }

                public BooleanSupplier getTransitionCheck() {
                        return transitionCheck;
                }

                public boolean checkTransition() {
                        return transitionCheck.getAsBoolean();
                }
        }

        private static final double popsicleDelay = 0.5;

        private List<Transition> transitions;
        private Timer stateTimer;
        private State currentState;
        public SuperstructureInputs inputs;

        // Constructor - Dependency Injection ile
        public SuperstructureO(Arm arm, Elevator elevator, Intake intake, RobotContainer robotContainer) {
                this.arm = arm;
                this.elevator = elevator;
                this.intake = intake;
                this.robot = robotContainer.robot;

                currentState = State.StartPosition;
                inputs = new SuperstructureInputs();
                stateTimer = new Timer();
                transitions = new ArrayList<>();
                initializeTransitions();
        }

        private void initializeTransitions() {
                // Starting transitions
                transitions.add(new Transition(State.StartPosition, State.Rest,
                                () -> inputs.wantGroundIntake || inputs.wantArmSourceIntake));
                transitions.add(new Transition(State.StartPosition, State.PreScore,
                                () -> robot.isAutonomous()));

                // Arm source intaking
                transitions.add(new Transition(State.Rest, State.ArmSourceIntake,
                                () -> inputs.wantArmSourceIntake));
                transitions.add(new Transition(State.ArmSourceIntake, State.Rest,
                                () -> !inputs.wantArmSourceIntake || arm.hasObject()));

                // Intake source intaking
                transitions.add(new Transition(State.Rest, State.SourceIntake,
                                () -> inputs.wantSourceIntake));
                transitions.add(new Transition(State.SourceIntake, State.Rest,
                                () -> !inputs.wantSourceIntake || intake.hasCoral()));

                // Trough reverse handoff
                transitions.add(new Transition(State.PreScore, State.Rest,
                                () -> inputs.wantedScoringLevel == ScoringLevel.TROUGH || !arm.hasObject()));
                transitions.add(new Transition(State.Rest, State.ReverseHandoff,
                                () -> arm.atSetpoint() && elevator.atSetpoint() &&
                                                inputs.wantedScoringLevel == ScoringLevel.TROUGH && arm.hasObject()
                                                && !intake.hasCoral() &&
                                                intake.getEffectivePivotState() == Intake.PivotState.Up
                                                && intake.atSetpoint()));
                transitions.add(new Transition(State.ReverseHandoff, State.Rest,
                                () -> intake.hasCoral() || inputs.wantResetSuperstructure));

                // Trough scoring
                transitions.add(new Transition(State.Rest, State.PreTrough,
                                () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.TROUGH &&
                                                elevator.atSetpoint() && arm.atSetpoint()));
                transitions.add(new Transition(State.PreTrough, State.Trough,
                                () -> intake.atSetpoint() && inputs.wantScore));
                transitions.add(new Transition(State.PreTrough, State.Rest,
                                () -> !inputs.wantExtend));
                transitions.add(new Transition(State.Trough, State.Rest,
                                () -> !inputs.wantScore));

                // Handoff transitions
                transitions.add(new Transition(State.Rest, State.PreHandoff,
                                () -> elevator.atSetpoint() && arm.atSetpoint()
                                                && inputs.wantedScoringLevel != ScoringLevel.TROUGH &&
                                                intake.hasCoral()));
                transitions.add(new Transition(State.PreHandoff, State.Handoff,
                                () -> elevator.atSetpoint() && arm.atSetpoint() && intake.atSetpoint()));
                transitions.add(new Transition(State.Handoff, State.Rest,
                                () -> arm.hasObject()));
                transitions.add(new Transition(State.Rest, State.PreScore,
                                () -> arm.hasObject() && inputs.wantedScoringLevel != ScoringLevel.TROUGH));
                transitions.add(new Transition(State.Handoff, State.Rest,
                                () -> inputs.wantResetSuperstructure));

                // L4 Scoring prep
                transitions.add(new Transition(State.PreScore, State.PrepareL4,
                                () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L4));
                transitions.add(new Transition(State.PreScore, State.PrepareL3,
                                () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L3));
                transitions.add(new Transition(State.PreScore, State.PrepareL2,
                                () -> inputs.wantExtend && inputs.wantedScoringLevel == ScoringLevel.L2));

                // Scoring transitions
                addScoringTransitions(ScoringLevel.L4, State.PrepareL4, State.StartL4, State.PlaceL4, State.AfterL4);
                addScoringTransitions(ScoringLevel.L3, State.PrepareL3, State.StartL3, State.PlaceL3, State.AfterL3);
                addScoringTransitions(ScoringLevel.L2, State.PrepareL2, State.StartL2, State.PlaceL2, State.AfterL2);

                // Algae removal
                transitions.add(new Transition(State.AlgaeExit, State.PreGetAlgae,
                                () -> inputs.wantGetAlgae && !arm.hasObject()));
                transitions.add(new Transition(State.Rest, State.PreGetAlgae,
                                () -> inputs.wantGetAlgae && !arm.hasObject()));
                transitions.add(new Transition(State.PreGetAlgae, State.Rest,
                                () -> !inputs.wantGetAlgae));
                transitions.add(new Transition(State.PreGetAlgae, State.GetAlgae,
                                () -> elevator.atSetpoint()));
                transitions.add(new Transition(State.GetAlgae, State.PreGetAlgae,
                                () -> !inputs.wantGetAlgae));
                transitions.add(new Transition(State.GetAlgae, State.PostGetAlgae,
                                () -> arm.hasObject()));
                transitions.add(new Transition(State.PostGetAlgae, State.AlgaeRest,
                                () -> arm.atSetpoint() && arm.atSafeReefDistance()));
                transitions.add(new Transition(State.AlgaeRest, State.AlgaeExit,
                                () -> !arm.hasObject()));
                transitions.add(new Transition(State.AlgaeExit, State.Rest,
                                () -> arm.atSetpoint() && elevator.atSetpoint()));

                // Barge scoring
                transitions.add(new Transition(State.AlgaeRest, State.PreBarge,
                                () -> inputs.wantExtend));
                transitions.add(new Transition(State.PreBarge, State.AlgaeRest,
                                () -> !inputs.wantExtend));
                transitions.add(new Transition(State.PreBarge, State.ScoreBarge,
                                () -> inputs.wantScore && swerve.atGoodScoringDistance()));
                transitions.add(new Transition(State.ScoreBarge, State.PreBarge,
                                () -> (!inputs.wantExtend || !arm.hasObject()) && arm.atSafeBargeDistance()));

                // Algae descore
                transitions.add(new Transition(State.Rest, State.AlgaeDescore,
                                () -> inputs.wantDescoreAlgae));
                transitions.add(new Transition(State.AlgaeDescore, State.Rest,
                                () -> !inputs.wantDescoreAlgae));

                // Processor scoring
                transitions.add(new Transition(State.AlgaeRest, State.PreProcessor,
                                () -> inputs.wantScoreProcessor));
                transitions.add(new Transition(State.PreProcessor, State.ScoreProcessor,
                                () -> inputs.wantScore));
                transitions.add(new Transition(State.ScoreProcessor, State.AlgaeRest,
                                () -> !inputs.wantScoreProcessor && !arm.hasObject() && arm.atSafeProcessorDistance()));
                transitions.add(new Transition(State.PreProcessor, State.AlgaeRest,
                                () -> !inputs.wantScoreProcessor && arm.atSafeProcessorDistance()));

                // Algae ground intake
                transitions.add(new Transition(State.Rest, State.PreAlgaeGroundIntake,
                                () -> inputs.wantAlgaeGroundIntake && !arm.hasObject()));
                transitions.add(new Transition(State.PreAlgaeGroundIntake, State.AlgaeGroundIntake,
                                () -> inputs.wantAlgaeGroundIntake && intake.atSetpoint()));
                transitions.add(new Transition(State.AlgaeGroundIntake, State.ExitAlgaeGroundIntake,
                                () -> !inputs.wantAlgaeGroundIntake || arm.hasObject()));
                transitions.add(new Transition(State.ExitAlgaeGroundIntake, State.AlgaeRest,
                                () -> elevator.atSetpoint() && arm.atSetpoint()));

                // Popsicle pickup
                transitions.add(new Transition(State.PopsiclePickup, State.PrePopsiclePickup,
                                () -> !inputs.wantPopsiclePickup
                                                || (robot.isAutonomous() && stateTimer.hasElapsed(popsicleDelay))));
                transitions.add(new Transition(State.PrePopsiclePickup, State.Rest,
                                () -> !inputs.wantPopsiclePickup));
                transitions.add(new Transition(State.Rest, State.PrePopsiclePickup,
                                () -> inputs.wantPopsiclePickup && !arm.hasObject()));
                transitions.add(new Transition(State.PrePopsiclePickup, State.PopsiclePickup,
                                () -> inputs.wantPopsiclePickup && intake.atSetpoint()));
                transitions.add(new Transition(State.PopsiclePickup, State.PreScore,
                                () -> stateTimer.hasElapsed(popsicleDelay) && arm.hasObject()));
        }

        private void addScoringTransitions(ScoringLevel scoringLevel, State prepare, State start, State place,
                        State after) {
                // Exit transitions
                transitions.add(new Transition(prepare, State.PreScore,
                                () -> arm.atSetpoint() && (!arm.hasObject() || !inputs.wantExtend
                                                || inputs.wantedScoringLevel != scoringLevel)));
                transitions.add(new Transition(start, prepare,
                                () -> !inputs.wantExtend || inputs.wantedScoringLevel != scoringLevel
                                                || !arm.hasObject()));

                // Normal transitions
                transitions.add(new Transition(prepare, start,
                                () -> elevator.lazierAtSetpoint() && arm.hasObject() && inputs.wantExtend
                                                && inputs.wantedScoringLevel == scoringLevel));

                transitions.add(new Transition(start, place, swerve::markPoseScored,
                                () -> elevator.atSetpoint() && arm.atSetpoint() && inputs.wantScore));

                transitions.add(new Transition(place, after,
                                () -> elevator.atSetpoint() && arm.atSetpoint() &&
                                                (place == State.PlaceL2 || place == State.PlaceL3
                                                                ? arm.atSafePlacementDistance()
                                                                : true)));

                transitions.add(new Transition(after, State.Rest,
                                () -> arm.insideFrame()));
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

        public State getState() {
                return currentState;
        }

        public void setState(State newState) {
                if (newState != currentState) {
                        currentState = newState;
                        stateTimer.restart();
                }
        }

        @Override
        public void periodic() {
                if (!elevator.isZeroed() || !intake.isZeroed() || !arm.isZeroed()) {
                        return;
                }

                stateTimer.start();

                for (Transition transition : transitions) {
                        if (transition.getCur() == currentState && transition.checkTransition()) {
                                setState(transition.getNext());
                                transition.getEnterFunction().run();
                                setStates();
                                return;
                        }
                }
        }

        public void setStates() {
                arm.setState(currentState.getArmPivot(), currentState.getArmRollers());
                elevator.setState(currentState.getElevator());
                intake.setState(currentState.getIntakePivot(), currentState.getIntakeRollers());
        }

        @Override
        public void initSendable(SendableBuilder builder) {
                builder.addStringProperty("State", () -> currentState.toString(), null);
                builder.addBooleanProperty("wantExtend", () -> inputs.wantExtend, null);
                builder.addBooleanProperty("wantScore", () -> inputs.wantScore, null);
                builder.addStringProperty("scoringLevel", () -> inputs.wantedScoringLevel.toString(), null);
        }

        // ... (State, ScoringLevel, SuperstructureInputs, Transition inner classes aynı
        // kalır)
}