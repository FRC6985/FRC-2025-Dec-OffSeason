package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
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
  public final XboxController driverController;
  public final XboxController operatorController;
  /*
   * PS5 Controller yerine artık ikinci XboxController kullanıyoruz.
   */

  // State
  public Superstructure.ScoringLevel lastScoringLevel = Superstructure.ScoringLevel.TROUGH;

  // Data class for drive inputs
  public static class DriveInputs {
    public final double forward;
    public final double left;
    public final double rotation;
    public final double deadzone;
    public final AlignMode alignMode;

    public DriveInputs(
        double forward, double left, double rotation, double deadzone, AlignMode alignMode) {
      this.forward = forward;
      this.left = left;
      this.rotation = rotation;
      this.deadzone = deadzone;
      this.alignMode = alignMode;
    }

    public boolean isNonZero() {
      return Math.abs(forward) > deadzone
          || Math.abs(left) > deadzone
          || Math.abs(rotation) > deadzone;
    }

    public DriveInputs redFlipped() {
      return new DriveInputs(-forward, -left, rotation, deadzone, alignMode);
    }
  }

  // Empty inputs constant
  public static final DriveInputs EMPTY_INPUTS =
      new DriveInputs(0.0, 0.0, 0.0, 0.0, AlignMode.None);

  // Constructor
  private Controls() {
    driverController = new XboxController(0);
    operatorController = new XboxController(1); // PS5 yerine 2. Xbox
  }

  // Driver inputs
  public DriveInputs getDriverInputs() {
    AlignMode alignMode;
    if (wantBargeAutoAlign()) {
      alignMode = AlignMode.BargeAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.ReefAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.TroughAlign;
    } else if (wantAlgaeAutoAlign()
        && getSuperstructureInputs().wantGetAlgae
        && !Arm.getInstance().isArmStuck) {
      alignMode = AlignMode.AlgaeAlign;
    } else {
      alignMode = AlignMode.None;
    }

    return new DriveInputs(
        driverController.getRawAxis(1), // left stick Y
        -driverController.getRawAxis(0), // left stick X
        -driverController.getRawAxis(4), // right stick X (turn)
        0.05,
        alignMode);
  }

  // Operator inputs — Xbox eşleştirme açıklamaları eklendi
  public DriveInputs getOperatorInputs() {
    AlignMode alignMode;
    if (wantBargeAutoAlign()) {
      alignMode = AlignMode.BargeAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel != Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.ReefAlign;
    } else if (wantCoralAutoAlign()
        && getSuperstructureInputs().wantedScoringLevel == Superstructure.ScoringLevel.TROUGH) {
      alignMode = AlignMode.TroughAlign;
    } else if (wantAlgaeAutoAlign()
        && getSuperstructureInputs().wantGetAlgae
        && !Arm.getInstance().isArmStuck) {
      alignMode = AlignMode.AlgaeAlign;
    } else {
      alignMode = AlignMode.None;
    }

    return new DriveInputs(
        -operatorController.getRawAxis(1), // left stick Y
        -operatorController.getRawAxis(0), // left stick X
        -operatorController.getRawAxis(4), // right stick X
        0.1,
        alignMode);
  }

  // Auto-align conditions
  public boolean wantCoralAutoAlign() {
    return getSuperstructureInputs().wantExtend;
  }

  public boolean wantAlgaeAutoAlign() {
    return getSuperstructureInputs().wantGetAlgae
        && Arm.getInstance().isAtSetpoint()
        && Elevator.getInstance().isAtSetpoint();
  }

  public boolean wantBargeAutoAlign() {
    return getSuperstructureInputs().wantExtend
        && (Superstructure.getInstance().state == Superstructure.State.AlgaeRest
            || Superstructure.getInstance().state == Superstructure.State.PreBarge
            || Superstructure.getInstance().state == Superstructure.State.ScoreBarge);
  }

  // Offset arm inputs
  public boolean wantOffsetArmPositive() {
    return operatorController.getRawAxis(0) > 0.9 && operatorController.getLeftStickButton();
  }

  public boolean wantOffsetArmNegative() {
    return operatorController.getRawAxis(0) < -0.9 && operatorController.getLeftStickButton();
  }

  // Superstructure inputs (PS5 tuşları → Xbox eşleştirmeleri yorumlandı)
  public Superstructure.SuperstructureInputs getSuperstructureInputs() {
    int pov = operatorController.getPOV();
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
        /*
         * PS5 L2 → Xbox Left Trigger (>0.5 threshold)
         */
        operatorController.getLeftTriggerAxis() > 0.5, // wantExtend

        /*
         * PS5 R2 → Xbox Right Trigger
         */
        operatorController.getRightTriggerAxis() > 0.5, // wantGroundIntake

        /*
         * PS5 Cross (X) → Xbox A
         */
        operatorController.getAButton(), // wantArmSourceIntake

        /*
         * PS5 Square → Xbox X
         */
        operatorController.getXButton(), // wantSourceIntake

        /*
         * Score: driver Right Bumper OR operator Right Stick Button
         * PS5 R3 → Xbox Right Stick Button
         */
        driverController.getRightBumper() || operatorController.getRightStickButton(), // wantScore
        level, // wantedScoringLevel

        /*
         * PS5 R1 → Xbox Right Bumper
         */
        operatorController.getRightBumper(), // wantGetAlgae

        /*
         * PS5 Triangle → Xbox Y
         */
        operatorController.getYButton(), // wantDescoreAlgae
        false, // wantVerticalPickup (PS5’de de kullanılmıyordu)

        /*
         * PS5 Options → Xbox Menu Button
         */
        operatorController.getStartButton(), // wantResetSuperstructure

        /*
         * PS5 Circle → Xbox B
         */
        operatorController.getBButton(), // wantScoreProcessor

        /*
         * PS5 L1 → Xbox Left Bumper
         */
        operatorController.getLeftBumper(), // wantAlgaeGroundIntake
        false // wantPopsiclePickup
        );
  }
}
