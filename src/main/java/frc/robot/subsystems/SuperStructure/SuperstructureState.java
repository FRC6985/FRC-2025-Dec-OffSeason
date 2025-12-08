package frc.robot.subsystems.SuperStructure;

import frc.robot.Constants.Elevator;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Intake;

public enum SuperstructureState {

        StartPosition(
                        Elevator.State.Down,
                        Arm.PivotState.Up, Arm.RollerState.SlowIdle,
                        Intake.PivotState.Up, Intake.RollerState.Off),

        Rest(
                        Elevator.State.PreHandoff,
                        Arm.PivotState.Down, Arm.RollerState.Idle),

        PrePopsiclePickup(
                        Elevator.State.PreHandoff,
                        Arm.PivotState.PopsiclePickup, Arm.RollerState.In,
                        Intake.PivotState.Down, Intake.RollerState.Off),

        PopsiclePickup(
                        Elevator.State.PopsiclePickup,
                        Arm.PivotState.PopsiclePickup, Arm.RollerState.In,
                        Intake.PivotState.Down, Intake.RollerState.Off),

        ArmSourceIntake(
                        Elevator.State.Down,
                        Arm.PivotState.Up, Arm.RollerState.In),

        SourceIntake(
                        Elevator.State.SourceIntake,
                        Arm.PivotState.Down, Arm.RollerState.Idle,
                        Intake.PivotState.Up, Intake.RollerState.In),

        PreHandoff(
                        Elevator.State.Handoff,
                        Arm.PivotState.Down, Arm.RollerState.In,
                        Intake.PivotState.Up, Intake.RollerState.In),

        Handoff(
                        Elevator.State.Handoff,
                        Arm.PivotState.Down, Arm.RollerState.In,
                        Intake.PivotState.Up, Intake.RollerState.Out),

        PreScore(
                        Elevator.State.PreScore,
                        Arm.PivotState.Up, Arm.RollerState.Idle,
                        Intake.PivotState.Up, Intake.RollerState.Off),

        ReverseHandoff(
                        Elevator.State.PreHandoff,
                        Arm.PivotState.Down, Arm.RollerState.Out,
                        Intake.PivotState.Up, Intake.RollerState.In),

        PreTrough(
                        Elevator.State.Trough,
                        Arm.PivotState.Down, Arm.RollerState.Idle,
                        Intake.PivotState.Trough, Intake.RollerState.Off),

        Trough(
                        Elevator.State.Trough,
                        Arm.PivotState.Down, Arm.RollerState.Idle,
                        Intake.PivotState.Trough, Intake.RollerState.TroughOut),

        // ---------- L4 ----------
        PrepareL4(
                        Elevator.State.L4,
                        Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),

        StartL4(
                        Elevator.State.L4,
                        Arm.PivotState.L4ScoreCoral, Arm.RollerState.SlowIdle),

        PlaceL4(
                        Elevator.State.ScoreL4,
                        Arm.PivotState.L4FinishScoreCoral, Arm.RollerState.Off),

        AfterL4(
                        Elevator.State.PreHandoff,
                        Arm.PivotState.Down, Arm.RollerState.SlowOut),

        // ---------- L3 ----------
        PrepareL3(
                        Elevator.State.L3,
                        Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),

        StartL3(
                        Elevator.State.L3,
                        Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),

        PlaceL3(
                        Elevator.State.ScoreL3,
                        Arm.PivotState.FinishScoreCoral, Arm.RollerState.Off),

        AfterL3(
                        Elevator.State.PostL3,
                        Arm.PivotState.Up, Arm.RollerState.SlowOut),

        // ---------- L2 ----------
        PrepareL2(
                        Elevator.State.L2,
                        Arm.PivotState.AboveScoreCoral, Arm.RollerState.Idle),

        StartL2(
                        Elevator.State.L2,
                        Arm.PivotState.ScoreCoral, Arm.RollerState.SlowIdle),

        PlaceL2(
                        Elevator.State.ScoreL2,
                        Arm.PivotState.FinishScoreCoral, Arm.RollerState.SlowOut),

        AfterL2(
                        Elevator.State.PostL2,
                        Arm.PivotState.Up, Arm.RollerState.Out),

        // ---------- Algae ----------
        PreGetAlgae(
                        Elevator.State.HighAlgae,
                        Arm.PivotState.SafeInsideRobotAngle, Arm.RollerState.In),

        GetAlgae(
                        Elevator.State.AutoAlgae,
                        Arm.PivotState.GetAlgae, Arm.RollerState.In),

        PostGetAlgae(
                        Elevator.State.AutoAlgae,
                        Arm.PivotState.PostAlgae, Arm.RollerState.AlgaeIdle),

        AlgaeRest(
                        Elevator.State.AlgaeRest,
                        Arm.PivotState.AlgaeUp, Arm.RollerState.AlgaeIdle),

        PreBarge(
                        Elevator.State.Barge,
                        Arm.PivotState.PreBarge, Arm.RollerState.AlgaeIdle),

        ScoreBarge(
                        Elevator.State.Barge,
                        Arm.PivotState.BargeScore, Arm.RollerState.Out),

        AlgaeDescore(
                        Elevator.State.AutoAlgae,
                        Arm.PivotState.DescoreAlgae, Arm.RollerState.Descore),

        AlgaeExit(
                        Elevator.State.PreHandoff,
                        Arm.PivotState.Down, Arm.RollerState.Out),

        PreProcessor(
                        Elevator.State.Processor,
                        Arm.PivotState.Processor, Arm.RollerState.AlgaeIdle),

        ScoreProcessor(
                        Elevator.State.Processor,
                        Arm.PivotState.Processor, Arm.RollerState.SlowOut),

        PreAlgaeGroundIntake(
                        Elevator.State.PreHandoff,
                        Arm.PivotState.AlgaeGroundPickup, Arm.RollerState.Off,
                        Intake.PivotState.Down, Intake.RollerState.Off),

        AlgaeGroundIntake(
                        Elevator.State.GroundAlgaeIntake,
                        Arm.PivotState.AlgaeGroundPickup, Arm.RollerState.In,
                        Intake.PivotState.Down, Intake.RollerState.Off),

        ExitAlgaeGroundIntake(
                        Elevator.State.PreHandoff,
                        Arm.PivotState.ExitAlgaeGroundPickup, Arm.RollerState.AlgaeIdle,
                        Intake.PivotState.Down, Intake.RollerState.Off);

        // State fields
        public final Elevator.State elevator;
        public final Arm.PivotState armPivot;
        public final Arm.RollerState armRollers;
        public final Intake.PivotState intakePivot;
        public final Intake.RollerState intakeRollers;

        SuperstructureState(
                        Elevator.State elevator,
                        Arm.PivotState armPivot,
                        Arm.RollerState armRollers) {
                this(elevator, armPivot, armRollers,
                                Intake.PivotState.OperatorControl,
                                Intake.RollerState.OperatorControl);
        }

        SuperstructureState(
                        Elevator.State elevator,
                        Arm.PivotState armPivot,
                        Arm.RollerState armRollers,
                        Intake.PivotState intakePivot,
                        Intake.RollerState intakeRollers) {
                this.elevator = elevator;
                this.armPivot = armPivot;
                this.armRollers = armRollers;
                this.intakePivot = intakePivot;
                this.intakeRollers = intakeRollers;
        }
}
