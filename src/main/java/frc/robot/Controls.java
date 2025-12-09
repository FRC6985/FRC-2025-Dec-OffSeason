package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;

public class Controls {
    private static final Controls INSTANCE = new Controls();

    public static Controls getInstance() {
        return INSTANCE;
    }

    // Enums
    public enum AlignMode {
        None,
        ReefAlign,
        TroughAlign,
        AlgaeAlign,
        BargeAlign
    }

    // Controllers
    private final CommandGenericHID driverController;
    public final CommandPS5Controller operatorController;

    // State
    public Superstructure.ScoringLevel lastScoringLevel = Superstructure.ScoringLevel.TROUGH;

    // Data class for drive inputs
    public static class DriveInputs {
        public final double forward;
        public final double left;
        public final double rotation;
        public final double deadzone;
        public final AlignMode alignMode;

        public DriveInputs(double forward, double left, double rotation, double deadzone, AlignMode alignMode) {
            this.forward = forward;
            this.left = left;
            this.rotation = rotation;
            this.deadzone = deadzone;
            this.alignMode = alignMode;
        }

        public boolean isNonZero() {
            return Math.abs(forward) > deadzone || Math.abs(left) > deadzone || Math.abs(rotation) > deadzone;
        }

        public DriveInputs redFlipped() {
            return new DriveInputs(-forward, -left, rotation, deadzone, alignMode);
        }
    }

    // Empty inputs constant
    public static final DriveInputs EMPTY_INPUTS = new DriveInputs(0.0, 0.0, 0.0, 0.0, AlignMode.None);

    // Constructor
    private Controls() {
        driverController = new CommandGenericHID(0);
        operatorController = new CommandPS5Controller(1);
    }

    // Driver inputs
    public DriveInputs getDriverInputs() {
        AlignMode alignMode;
        if (wantBargeAutoAlign()) {
            alignMode = AlignMode.BargeAlign;
        } else if (wantCoralAutoAlign() &&
                getSuperstructureInputs().wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) {
            alignMode = AlignMode.ReefAlign;
        } else if (wantCoralAutoAlign() &&
                getSuperstructureInputs().wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) {
            alignMode = AlignMode.TroughAlign;
        } else if (wantAlgaeAutoAlign() &&
                getSuperstructureInputs().wantGetAlgae &&
                !Arm.getInstance().isArmStuck) {
            alignMode = AlignMode.AlgaeAlign;
        } else {
            alignMode = AlignMode.None;
        }

        return new DriveInputs(
                driverController.getRawAxis(2),
                -driverController.getRawAxis(3),
                -driverController.getRawAxis(0),
                0.05,
                alignMode);
    }

    // Operator inputs
    public DriveInputs getOperatorInputs() {
        AlignMode alignMode;
        if (wantBargeAutoAlign()) {
            alignMode = AlignMode.BargeAlign;
        } else if (wantCoralAutoAlign() &&
                getSuperstructureInputs().wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) {
            alignMode = AlignMode.ReefAlign;
        } else if (wantCoralAutoAlign() &&
                getSuperstructureInputs().wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) {
            alignMode = AlignMode.TroughAlign;
        } else if (wantAlgaeAutoAlign() &&
                getSuperstructureInputs().wantGetAlgae &&
                !Arm.getInstance().isArmStuck) {
            alignMode = AlignMode.AlgaeAlign;
        } else {
            alignMode = AlignMode.None;
        }

        return new DriveInputs(
                -operatorController.getHID().getLeftY(),
                -operatorController.getHID().getLeftX(),
                -operatorController.getHID().getRightX(),
                0.1,
                alignMode);
    }

    // Auto-align conditions
    public boolean wantCoralAutoAlign() {
        return getSuperstructureInputs().wantExtend;
    }

    public boolean wantAlgaeAutoAlign() {
        return getSuperstructureInputs().wantGetAlgae &&
                Arm.getInstance().isAtSetpoint() &&
                Elevator.getInstance().isAtSetpoint();
    }

    public boolean wantBargeAutoAlign() {
        return getSuperstructureInputs().wantExtend &&
                (Superstructure.getInstance().state == Superstructure.State.AlgaeRest ||
                        Superstructure.getInstance().state == Superstructure.State.PreBarge ||
                        Superstructure.getInstance().state == Superstructure.State.ScoreBarge);
    }

    // Offset arm inputs
    public boolean wantOffsetArmPositive() {
        return operatorController.getHID().getLeftX() > 0.9 && operatorController.getHID().getL3Button();
    }

    public boolean wantOffsetArmNegative() {
        return operatorController.getHID().getLeftX() < -0.9 && operatorController.getHID().getL3Button();
    }

    // Superstructure inputs
    public Superstructure.SuperstructureInputs getSuperstructureInputs() {
        int pov = operatorController.getHID().getPOV();
        Superstructure.ScoringLevel level;

        switch (pov) {
            case 0:
                level = Superstructure.ScoringLevel.L4;
                break;
            case 270:
                level = Superstructure.ScoringLevel.L3;
                break;
            case 180:
                level = Superstructure.ScoringLevel.L2;
                break;
            case 90:
                level = Superstructure.ScoringLevel.TROUGH;
                break;
            default:
                level = lastScoringLevel;
        }
        lastScoringLevel = level;

        return new Superstructure.SuperstructureInputs(
                operatorController.getHID().getL2Button(), // wantExtend
                operatorController.getHID().getR2Button(), // wantGroundIntake
                operatorController.getHID().getCrossButton(), // wantArmSourceIntake
                operatorController.getHID().getSquareButton(), // wantSourceIntake
                driverController.getRawAxis(4) > 0.5 || operatorController.getHID().getR3Button(), // wantScore
                level, // wantedScoringLevel
                operatorController.getHID().getR1Button(), // wantGetAlgae
                operatorController.getHID().getTriangleButton(), // wantDescoreAlgae
                false, // wantVerticalPickup
                operatorController.getHID().getOptionsButton(), // wantResetSuperstructure
                operatorController.getHID().getCircleButton(), // wantScoreProcessor
                operatorController.getHID().getL1Button(), // wantAlgaeGroundIntake
                false // wantPopsiclePickup
        );
    }
}